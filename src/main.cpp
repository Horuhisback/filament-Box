// FilamentBox: merged firmware. GPIOs and active-LOW sequences from main.cpp.
#include <Arduino.h>
#include <ArduinoJson.h>
#include <DHT.h>
#include <HTTPClient.h>
#include <LittleFS.h>
#include <Preferences.h>
#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <WiFiManager.h>
#include <mbedtls/md.h>
#include <esp_partition.h>
#include <time.h>
#include <vector>
#include <algorithm>
#include <stdarg.h>
#include "../include/config.h"
#include "../include/google_roots.h"
#include "../include/WifiRetry.h"

constexpr int UV_PIN=1, FAN_PIN=2;
constexpr int DHT_PINS[]={3,12,13}; // chamber, outdoor, post-silica
constexpr int motor1Pins[]={4,5,6,7}, motor2Pins[]={8,9,10,11};
constexpr bool HALF_STEP=false;
constexpr uint32_t SENSOR_MS=10000, CONTROL_MS=5000, LEASE_MS=30000;
constexpr uint32_t WIFI_PORTAL_AFTER_MS=10000, WIFI_PORTAL_SECONDS=180;
constexpr uint32_t WIFI_BOOT_TIMEOUT_SECONDS=3, WIFI_SAVE_TIMEOUT_SECONDS=8;
constexpr uint32_t WIFI_DEFAULT_TIMEOUT_MS=15000;
constexpr char WIFI_SETUP_SSID[]="FilamentBox-Setup";
constexpr size_t MAX_PENDING=128, PACKET_BYTES=2304; // 21 min 20 s at 10 s/sample
constexpr int STEP_MS=HALF_STEP ? 3 : 6;
constexpr int fullSequence[4][4]={{0,0,1,1},{1,0,0,1},{1,1,0,0},{0,1,1,0}};
constexpr int halfSequence[8][4]={{0,1,1,1},{0,0,1,1},{1,0,1,1},{1,0,0,1},
  {1,1,0,1},{1,1,0,0},{1,1,1,0},{0,1,1,0}};

struct Control {
  bool power=false, automatic=true, fan=false, inletOpen=false, outletExterior=false;
  float target=30, hysteresis=5, uvAlert=3;
  char mode[12]="OFF";
  uint32_t receivedAt=0;
};
struct Packet { char json[PACKET_BYTES]; };
struct NetworkLog { char text[192]; };
struct Motor { const int *pins; int phase=0, remaining=0, direction=1; uint32_t last=0; };
Motor motors[]={{motor1Pins},{motor2Pins}};
DHT dht[]={DHT(3,DHT11),DHT(12,DHT11),DHT(13,DHT11)};
Control control;
QueueHandle_t cloudCommands, latestTelemetry, networkLogs;
SemaphoreHandle_t fsMutex;
bool fsReady=false, fanRunning=false, chamberValid=false, fanDemand=false, localControl=false;
float chamberRh=NAN, uvRaw=0, uvMv=0;
uint32_t lastSensor=0, lastValidChamber=0, lastUsb=0, bootNumber=0, sampleNumber=0, dropped=0;
String bootNonce, serialBuffer;
uint32_t serialSequence=0, responseSequence=0;
bool serialOverflow=false;

bool usbSessionActive() { return lastUsb && millis()-lastUsb<15000; }

void networkLog(const char *format, ...) {
  NetworkLog entry{};
  va_list args; va_start(args,format);
  vsnprintf(entry.text,sizeof(entry.text),format,args); va_end(args);
  xQueueSend(networkLogs,&entry,0); // bounded; never wait for a serial monitor
}
void networkLogTask() {
  NetworkLog entry;
  // Only loop() writes runtime diagnostics, so JSON frames cannot interleave.
  // Keep application USB sessions dedicated to authenticated protocol messages.
  if(usbSessionActive()) return;
  if(Serial && xQueueReceive(networkLogs,&entry,0)==pdTRUE) Serial.println(entry.text);
}

void stopMotors() {
  for (auto &m : motors) { m.remaining=0; for(int i=0;i<4;i++) digitalWrite(m.pins[i],HIGH); }
}
void setFan(bool on) { fanRunning=on; digitalWrite(FAN_PIN,on ? HIGH : LOW); }
void safeStop() { setFan(false); fanDemand=false; stopMotors(); }
bool validControl(JsonVariantConst v, Control &next) {
  if (!v.is<JsonObjectConst>()) return false;
  for(const char *key : {"power","auto","fan","inletOpen","outletExterior"}) if(!v[key].is<bool>()) return false;
  if(!v["mode"].is<const char *>()) return false;
  String mode=v["mode"].as<String>();
  if(mode!="OFF" && mode!="AUTO" && mode!="STORAGE" && mode!="RECIRC" && mode!="PURGE" && mode!="MAINT") return false;
  auto s=v["settings"];
  for(const char *key : {"targetRh","hysteresis","uvAlert"}) if(!s[key].is<float>() || !isfinite(s[key].as<float>())) return false;
  float target=s["targetRh"], hyst=s["hysteresis"], uv=s["uvAlert"];
  if(target<10 || target>60 || hyst<1 || hyst>20 || uv<0 || uv>11) return false;
  next.power=v["power"]; next.automatic=v["auto"]; next.fan=v["fan"];
  next.inletOpen=v["inletOpen"]; next.outletExterior=v["outletExterior"];
  next.target=target; next.hysteresis=hyst; next.uvAlert=uv;
  strlcpy(next.mode,mode.c_str(),sizeof(next.mode)); next.receivedAt=millis(); return true;
}
void motorTask() {
  // No automatic positioning without homing. MOTOR_TEST is a bounded bench test.
  for(auto &m : motors) {
    if(!m.remaining || millis()-m.last<STEP_MS) continue;
    m.last=millis(); int count=HALF_STEP ? 8 : 4;
    m.phase=(m.phase+m.direction+count)%count;
    for(int i=0;i<4;i++) digitalWrite(m.pins[i],HALF_STEP ? halfSequence[m.phase][i] : fullSequence[m.phase][i]);
    if(--m.remaining==0) for(int i=0;i<4;i++) digitalWrite(m.pins[i],HIGH);
  }
}
void controlTask() {
  Control incoming;
  if(xQueueReceive(cloudCommands,&incoming,0)==pdTRUE && !localControl) control=incoming;
  if(localControl && (!lastUsb || millis()-lastUsb>=15000)) { localControl=false; control.power=false; safeStop(); }
  if(!control.receivedAt || millis()-control.receivedAt>LEASE_MS) { safeStop(); return; }
  if(control.automatic) {
    if(!chamberValid || millis()-lastValidChamber>2*SENSOR_MS) fanDemand=false;
    else if(chamberRh>=control.target+control.hysteresis) fanDemand=true;
    else if(chamberRh<=control.target) fanDemand=false;
  } else fanDemand=control.fan;
  setFan(control.power && String(control.mode)!="OFF" && fanDemand);
}
void uvTask() {
  static uint32_t last=0,rawSum=0,mvSum=0;
  static int count=0,previousLoad=-1;
  int load=(motors[0].remaining ? 1:0)|(motors[1].remaining ? 2:0)|(fanRunning ? 4:0);
  if(load!=previousLoad) { rawSum=mvSum=count=0; previousLoad=load; }
  if(millis()-last<15) return;
  last=millis(); rawSum+=analogRead(UV_PIN); mvSum+=analogReadMilliVolts(UV_PIN);
  if(++count==32) { uvRaw=rawSum/32.0f; uvMv=mvSum/32.0f; rawSum=mvSum=count=0; }
}
uint32_t crc32(const String &data) {
  uint32_t crc=0xffffffff;
  for(size_t i=0;i<data.length();i++) { crc^=(uint8_t)data[i];
    for(int b=0;b<8;b++) crc=(crc>>1)^(0xedb88320UL & (0-(crc&1))); }
  return ~crc;
}
std::vector<String> pendingFiles() { // caller holds fsMutex
  std::vector<String> files; File dir=LittleFS.open("/");
  for(File f=dir.openNextFile();f;f=dir.openNextFile()) {
    String name=f.name(); if(!name.startsWith("/")) name="/"+name;
    if(name.endsWith(".json")) files.push_back(name);
    f.close();
  }
  std::sort(files.begin(),files.end()); return files;
}
bool cacheSample(const String &id,const String &payload) {
  if(!fsReady) return false;
  xSemaphoreTake(fsMutex,portMAX_DELAY);
  auto files=pendingFiles();
  if(files.size()>=MAX_PENDING) { LittleFS.remove(files.front()); dropped++; }
  String path="/"+id+".json",temporary="/pending.tmp";
  File file=LittleFS.open(temporary,"w"); String record=String(crc32(payload),HEX)+"\n"+payload;
  bool ok=file && file.print(record)==record.length(); file.close();
  if(ok) ok=LittleFS.rename(temporary,path);
  if(!ok) LittleFS.remove(temporary);
  xSemaphoreGive(fsMutex); return ok;
}
String mac(const String &data) {
  uint8_t result[32]; char hex[65];
  mbedtls_md_hmac(mbedtls_md_info_from_type(MBEDTLS_MD_SHA256),(const uint8_t*)SERIAL_PSK,
    strlen(SERIAL_PSK),(const uint8_t*)data.c_str(),data.length(),result);
  for(int i=0;i<32;i++) snprintf(hex+2*i,3,"%02x",result[i]);
  return String(hex);
}
bool equalMac(const String &a,const String &b) {
  if(a.length()!=64 || b.length()!=64) return false;
  uint8_t diff=0; for(int i=0;i<64;i++) diff|=a[i]^b[i]; return diff==0;
}
void sendSerial(JsonDocument &body) {
  if(!usbSessionActive() || strlen(SERIAL_PSK)<32) return;
  String payload; serializeJson(body,payload);
  JsonDocument frame; frame["v"]=1; frame["boot"]=bootNonce;
  frame["seq"]=++responseSequence; frame["payload"]=payload;
  frame["mac"]=mac("D|"+bootNonce+"|"+String(responseSequence)+"|"+payload);
  if(Serial.availableForWrite()>0) { serializeJson(frame,Serial); Serial.println(); }
}
void addControl(JsonDocument &doc) {
  doc["power"]=control.power; doc["auto"]=control.automatic; doc["fan"]=fanRunning;
  doc["inletOpen"]=false; doc["outletExterior"]=false; doc["positionKnown"]=false;
  doc["mode"]=control.mode; doc["settings"]["targetRh"]=control.target;
  doc["settings"]["hysteresis"]=control.hysteresis; doc["settings"]["uvAlert"]=control.uvAlert;
}
void serialTask() {
  // Bound work per loop; discard an oversized frame, including its suffix.
  for(int budget=0;Serial.available() && budget<256;budget++) {
    char c=Serial.read(); if(c=='\r') continue;
    if(c!='\n') {
      if(serialBuffer.length()<4096 && !serialOverflow) serialBuffer+=c;
      else { serialOverflow=true; serialBuffer=""; } continue;
    }
    String line=serialBuffer; serialBuffer="";
    if(serialOverflow) { serialOverflow=false; continue; }
    if(line=="HELLO") {
      char challenge[33];
      snprintf(challenge,sizeof(challenge),"%08lx%08lx%08lx%08lx",(unsigned long)esp_random(),(unsigned long)esp_random(),(unsigned long)esp_random(),(unsigned long)esp_random());
      bootNonce=challenge; serialSequence=responseSequence=0;
      Serial.printf("{\"v\":1,\"hello\":\"%s\"}\n",bootNonce.c_str()); continue;
    }
    if(strlen(SERIAL_PSK)<32) continue;
    JsonDocument frame,body;
    if(deserializeJson(frame,line) || frame["v"]!=1 || frame["boot"].as<String>()!=bootNonce ||
       !frame["seq"].is<uint32_t>() || !frame["payload"].is<const char *>()) continue;
    uint32_t seq=frame["seq"]; String payload=frame["payload"].as<String>();
    if(seq<=serialSequence || !equalMac(frame["mac"].as<String>(),mac("A|"+bootNonce+"|"+String(seq)+"|"+payload))) continue;
    serialSequence=seq; if(deserializeJson(body,payload)) continue;
    lastUsb=millis(); String cmd=body["cmd"] | ""; bool ok=true; String reason="OK";
    if(cmd=="GET_STATUS") { if(localControl) control.receivedAt=millis(); }
    else if(cmd=="STOP") { localControl=true; control.power=false; control.receivedAt=millis(); safeStop(); }
    else if(cmd=="SET_CONTROL") {
      Control next;
      if(!validControl(body["control"],next)) { ok=false; reason="INVALID_CONTROL"; control.power=false; safeStop(); }
      else { control=next; localControl=true; }
    } else if(cmd=="MOTOR_TEST") {
      int index=body["motor"] | 0,direction=body["direction"] | 0;
      if(index<1 || index>2 || (direction!=1 && direction!=-1) ||
         !body["benchConfirmed"].is<bool>() || !body["benchConfirmed"].as<bool>()) { ok=false; reason="INVALID_BENCH_TEST"; }
      else {
        safeStop(); localControl=true; control.power=false; control.receivedAt=millis();
        motors[index-1].direction=direction; motors[index-1].remaining=(HALF_STEP ? 4096:2048)/4;
      }
    } else { ok=false; reason="UNKNOWN_COMMAND"; }
    JsonDocument reply; reply["type"]="ACK"; reply["ack"]=seq; reply["ok"]=ok; reply["reason"]=reason;
    addControl(reply); sendSerial(reply);
    if(cmd=="GET_STATUS") {
      Packet latest;
      if(xQueuePeek(latestTelemetry,&latest,0)==pdTRUE) {
        JsonDocument status; if(!deserializeJson(status,latest.json)) { addControl(status); status["type"]="STATUS"; sendSerial(status); }
      }
    }
  }
}
void sampleSensors() {
  JsonDocument doc; const char *names[]={"chamber","outdoor","postSilica"};
  const char *sensorLabels[]={"Camara","Exterior","Salida de silica"};
  const bool printReadings=!usbSessionActive() && Serial && Serial.availableForWrite()>0;
  JsonArray alerts=doc["alerts"].to<JsonArray>();
  for(int i=0;i<3;i++) {
    float t=dht[i].readTemperature(),rh=dht[i].readHumidity();
    bool valid=isfinite(t) && isfinite(rh) && t>=0 && t<=50 && rh>=0 && rh<=100;
    doc[names[i]]["valid"]=valid;
    if(valid) { doc[names[i]]["t"]=t; doc[names[i]]["rh"]=rh; }
    else { doc[names[i]]["t"]=nullptr; doc[names[i]]["rh"]=nullptr; alerts.add(String("DHT11 GPIO ")+DHT_PINS[i]+": sin lectura valida"); }
    if(printReadings) {
      if(valid) Serial.printf("[SENSOR] %s (DHT11 GPIO %d): %.1f C | %.1f %% HR\n",sensorLabels[i],DHT_PINS[i],t,rh);
      else Serial.printf("[SENSOR] %s (DHT11 GPIO %d): sin lectura valida\n",sensorLabels[i],DHT_PINS[i]);
    }
    if(i==0) { chamberValid=valid; chamberRh=valid ? rh : NAN; if(valid) lastValidChamber=millis(); }
  }
  if(printReadings) Serial.printf("[SENSOR] UV (GPIO %d): %.3f V | ADC %.1f\n",UV_PIN,uvMv/1000.0f,uvRaw);
  controlTask(); doc["uv"]=uvMv/1000.0f; doc["uvUnit"]="V"; doc["uvRaw"]=uvRaw;
  doc["silica"]="SIN_CALIBRAR"; doc["wifi"]=WiFi.status()==WL_CONNECTED ? "Conectado":"Desconectado";
  doc["sampleUptimeMs"]=millis(); time_t now=time(nullptr);
  doc["sampleTs"]=now>1700000000 ? (int64_t)now*1000 : 0; doc["clockSynced"]=now>1700000000;
  char id[40]; snprintf(id,sizeof(id),"%010lu-%010lu",(unsigned long)bootNumber,(unsigned long)++sampleNumber);
  doc["sampleId"]=id; doc["droppedSamples"]=dropped; doc["queueCapacity"]=MAX_PENDING; addControl(doc);
  String payload; serializeJson(doc,payload);
  if(!cacheSample(id,payload)) { alerts.add("No se pudo guardar la lectura en flash"); payload=""; serializeJson(doc,payload); }
  Packet packet{};
  if(payload.length()<sizeof(packet.json)) { payload.toCharArray(packet.json,sizeof(packet.json)); xQueueOverwrite(latestTelemetry,&packet); }
  doc["type"]="STATUS"; sendSerial(doc);
}

// HTTP/TLS runs in a worker so outages do not block sampling, USB or motor steps.
void networkTask(void *) {
  WiFiManager manager;
  WiFi.onEvent([](WiFiEvent_t,WiFiEventInfo_t info) {
    networkLog("[WIFI] Disconnected; reason=%u. Check 2.4 GHz, password, signal and DHCP.",
      (unsigned)info.wifi_sta_disconnected.reason);
  },ARDUINO_EVENT_WIFI_STA_DISCONNECTED);
  // One reconnect owner: never let the core race WiFiManager or this worker.
  WiFi.setAutoReconnect(false); WiFi.setSleep(true);
  const bool hasDefaultWifi=WIFI_DEFAULT_SSID[0]!='\0';
  auto connectDefaultWifi=[]() {
    // Start with RAM storage while STA is off to preserve portal credentials.
    WiFi.persistent(false);
    bool started=WiFi.mode(WIFI_STA);
    WiFi.persistent(true); // The next driver initialization can load saved credentials.
    if(!started) return false;
    networkLog("[WIFI] Trying configured default network for up to 15 s.");
    WiFi.begin(WIFI_DEFAULT_SSID,WIFI_DEFAULT_PASSWORD);
    uint32_t start=millis();
    while(WiFi.status()!=WL_CONNECTED && millis()-start<WIFI_DEFAULT_TIMEOUT_MS) {
      vTaskDelay(pdMS_TO_TICKS(50));
    }
    if(WiFi.status()==WL_CONNECTED) return true;
    networkLog("[WIFI] Default network timed out; status=%d. Trying saved settings next.",WiFi.status());
    WiFi.enableSTA(false);
    return false;
  };
  manager.setDebugOutput(false); manager.setConfigPortalBlocking(false);
  manager.setConfigPortalTimeout(WIFI_PORTAL_SECONDS);
  manager.setConnectTimeout(WIFI_BOOT_TIMEOUT_SECONDS);
  manager.setSaveConnectTimeout(WIFI_SAVE_TIMEOUT_SECONDS);
  manager.setConnectRetries(1); manager.setWiFiAutoReconnect(false);
  manager.setShowPassword(false); manager.setShowInfoUpdate(false);
  manager.setAPStaticIPConfig(IPAddress(192,168,4,1),IPAddress(192,168,4,1),IPAddress(255,255,255,0));
  manager.setAPCallback([](WiFiManager *) {
    networkLog("[WIFI] Setup network: %s | http://192.168.4.1 | open for 180 s; use the configured setup password.",WIFI_SETUP_SSID);
  });
  manager.setPreSaveConfigCallback([]() {
    // Stop an earlier attempt before WiFiManager changes station credentials.
    // Disabling STA preserves the setup AP and does not erase saved networks.
    WiFi.setAutoReconnect(false);
    WiFi.enableSTA(false);
    networkLog("[WIFI] Testing submitted network (up to 8 s); credentials hidden.");
  });
  manager.setSaveConfigCallback([]() { networkLog("[WIFI] Network settings saved."); });
  const char *menu[]={"wifi"}; manager.setMenu(menu,1);
  bool setupEnabled=strlen(WIFI_SETUP_PASSWORD)>=12;
  if(!hasDefaultWifi || !connectDefaultWifi()) {
    WiFi.mode(WIFI_STA);
    if(setupEnabled) {
      networkLog("[WIFI] Trying saved network for up to 3 s; setup portal follows on failure.");
      manager.autoConnect(WIFI_SETUP_SSID,WIFI_SETUP_PASSWORD);
    } else {
      networkLog("[WIFI] Setup disabled: configure a password of at least 12 characters.");
      WiFi.begin();
    }
  }
  configTime(0,0,"time.google.com","pool.ntp.org");
  String token,lastPublished; uint32_t tokenUntil=0,lastAuth=0,lastPoll=0,lastSend=0;
  bool authAttempted=false,wasConnected=false;
  bool retryDefault=hasDefaultWifi;
  uint32_t disconnectedSince=millis(),lastPortalAttempt=millis(),lastHeartbeat=0;
  int previousStatus=-1;
  int64_t acceptedIssuedAt=0;
  WifiRetry wifiRetry(millis());
  if(!manager.getConfigPortalActive() && WiFi.status()!=WL_CONNECTED) wifiRetry.start(millis());
  auto request=[&](const String &url,const char *method,const String &body,String &response) {
    if(WiFi.status()!=WL_CONNECTED || time(nullptr)<1700000000 || !url.startsWith("https://")) return -1;
    WiFiClientSecure client; client.setCACert(GOOGLE_ROOTS); client.setHandshakeTimeout(5);
    HTTPClient http; http.setConnectTimeout(5000); http.setTimeout(5000);
    if(!http.begin(client,url)) return -1;
    http.addHeader("Content-Type","application/json");
    int code=String(method)=="GET" ? http.GET() : http.sendRequest(method,body);
    response=http.getString(); http.end(); if(code==401 || code==403) token=""; return code;
  };
  auto dbRequest=[&](const String &path,const char *method,const String &body,String &response) {
    String url=String(FIREBASE_DATABASE_URL)+"/filamentBox/"+path+".json?auth="+token;
    int code=request(url,method,body,response);
    if(code==401 || code==403) {
      networkLog("[FIREBASE] Database rejected %s /filamentBox/%s: HTTP %d. Session cleared.",
        method,path.c_str(),code);
    }
    return code;
  };
  for(;;) {
    bool portalWasActive=manager.getConfigPortalActive();
    manager.process();
    uint32_t now=millis();
    int status=WiFi.status();
    bool portalActive=manager.getConfigPortalActive();
    if(status!=previousStatus) {
      networkLog("[WIFI] Status=%d | setup=%s",status,portalActive ? "ON":"OFF");
      previousStatus=status;
    }
    if(status!=WL_CONNECTED) {
      acceptedIssuedAt=0;
      if(wasConnected) {
        disconnectedSince=lastPortalAttempt=now;
        wifiRetry.reset(now);
        retryDefault=hasDefaultWifi;
      }
      wasConnected=false;
      if(portalWasActive && !portalActive) {
        // After timeout, try STA again before opening another setup window.
        disconnectedSince=lastPortalAttempt=now;
        wifiRetry.reset(now);
        retryDefault=hasDefaultWifi;
        networkLog("[WIFI] Setup closed; retrying the saved network before reopening in 10 s.");
      }
      if(portalActive) {
        // Manager owns scanning and credential submission for the entire portal
        // window, even before a phone attaches. No background begin/reconnect.
        wifiRetry.reset(now);
        vTaskDelay(pdMS_TO_TICKS(50)); continue;
      }
      if(wifiRetry.expired(now)) {
        WiFi.enableSTA(false); // cancel the driver operation before another begin
        wifiRetry.failed(now);
        networkLog("[WIFI] Attempt ended; next retry after %lu ms.",(unsigned long)wifiRetry.delay());
      }
      if(setupEnabled && now-disconnectedSince>=WIFI_PORTAL_AFTER_MS &&
         now-lastPortalAttempt>=WIFI_PORTAL_AFTER_MS) {
        lastPortalAttempt=now;
        wifiRetry.reset(now);
        manager.startConfigPortal(WIFI_SETUP_SSID,WIFI_SETUP_PASSWORD);
        vTaskDelay(pdMS_TO_TICKS(50)); continue;
      }
      if(wifiRetry.ready(now,false,false)) {
        if(retryDefault) {
          retryDefault=false;
          if(WiFi.enableSTA(false) && connectDefaultWifi()) continue;
        }
        wifiRetry.start(millis());
        // Recreate only STA, retaining credentials. No immediate fallback call
        // to begin() after a failed reconnect(), and no auto-reconnect in parallel.
        if(WiFi.enableSTA(false) && WiFi.enableSTA(true)) {
          WiFi.setAutoReconnect(false);
          WiFi.begin();
          networkLog("[WIFI] Started one saved-network attempt (up to 8 s).");
        } else {
          wifiRetry.failed(millis());
          networkLog("[WIFI] Unable to restart STA; waiting before retry.");
        }
      }
      vTaskDelay(pdMS_TO_TICKS(50)); continue;
    }
    if(!wasConnected) {
      networkLog("[WIFI] Connected: IP=%s RSSI=%d dBm; checking clock and Firebase next.",
        WiFi.localIP().toString().c_str(),WiFi.RSSI());
    }
    wasConnected=true; wifiRetry.reset(now);
    if(manager.getConfigPortalActive()) manager.stopConfigPortal();
    if(now-lastHeartbeat>=10000) {
      lastHeartbeat=now;
      networkLog("[NETWORK] WiFi=connected | clock=%s | Firebase=%s",
        time(nullptr)>1700000000 ? "ready":"waiting for NTP",
        token.isEmpty() ? "authentication pending":"authenticated");
    }
    if(time(nullptr)<1700000000) { vTaskDelay(pdMS_TO_TICKS(100)); continue; }
    if(token.isEmpty() || (int32_t)(millis()-tokenUntil)>=0) {
      if((!authAttempted || millis()-lastAuth>=30000) && strlen(FIREBASE_EMAIL) && strlen(FIREBASE_PASSWORD)) {
        lastAuth=millis(); authAttempted=true;
        JsonDocument auth; auth["email"]=FIREBASE_EMAIL; auth["password"]=FIREBASE_PASSWORD; auth["returnSecureToken"]=true;
        String body,response; serializeJson(auth,body);
        int code=request(String("https://identitytoolkit.googleapis.com/v1/accounts:signInWithPassword?key=")+FIREBASE_API_KEY,"POST",body,response);
        JsonDocument result;
        if(code==200 && !deserializeJson(result,response) && result["idToken"].is<const char *>()) {
          token=result["idToken"].as<String>(); tokenUntil=millis()+3300000UL;
          networkLog("[FIREBASE] Authentication successful.");
        } else {
          networkLog("[FIREBASE] Authentication failed: HTTP %d. WiFi is connected; check Internet and Firebase credentials.",code);
        }
      }
      vTaskDelay(pdMS_TO_TICKS(50)); continue;
    }
    if(millis()-lastPoll>=CONTROL_MS) {
      lastPoll=millis(); String response; int code=dbRequest("control","GET","",response);
      JsonDocument doc; Control next;
      if(code==200 && !deserializeJson(doc,response) && validControl(doc.as<JsonVariantConst>(),next)) {
        int64_t issued=doc["issuedAt"] | (int64_t)0,age=(int64_t)time(nullptr)*1000-issued;
        if(issued>0 && (issued==acceptedIssuedAt || (age>=-5000 && age<=LEASE_MS))) {
          acceptedIssuedAt=issued; xQueueOverwrite(cloudCommands,&next);
        }
      } else { acceptedIssuedAt=0; next.power=false; next.receivedAt=millis(); xQueueOverwrite(cloudCommands,&next); }
    }
    if(millis()-lastSend>=5000 && !token.isEmpty()) {
      lastSend=millis(); String response; Packet latest;
      if(xQueuePeek(latestTelemetry,&latest,0)==pdTRUE) {
        JsonDocument doc;
        if(!deserializeJson(doc,latest.json)) {
          String sampleId=doc["sampleId"].as<String>();
          if(sampleId!=lastPublished) {
            doc["receivedAt"][".sv"]="timestamp"; String body; serializeJson(doc,body);
            int code=dbRequest("sensores","PUT",body,response);
            if(code>=200 && code<300) lastPublished=sampleId;
          }
        }
      }
      if(fsReady && !token.isEmpty()) {
        String path,record;
        xSemaphoreTake(fsMutex,portMAX_DELAY); auto files=pendingFiles();
        if(!files.empty()) { path=files.front(); File f=LittleFS.open(path); record=f.readString(); f.close(); }
        xSemaphoreGive(fsMutex);
        if(!path.isEmpty()) {
          int split=record.indexOf('\n'); String payload=record.substring(split+1);
          bool intact=split>0 && record.substring(0,split)==String(crc32(payload),HEX);
          if(!intact) {
            xSemaphoreTake(fsMutex,portMAX_DELAY);
            // Keep one damaged record for diagnostics; bound quarantine storage.
            LittleFS.remove("/corrupt.bad"); LittleFS.rename(path,"/corrupt.bad");
            xSemaphoreGive(fsMutex);
          } else {
            String key=path.substring(1,path.length()-5);
            int code=dbRequest("muestras/"+key,"PUT",payload,response);
            if(code>=200 && code<300) { xSemaphoreTake(fsMutex,portMAX_DELAY); LittleFS.remove(path); xSemaphoreGive(fsMutex); }
          }
        }
      }
    }
    vTaskDelay(pdMS_TO_TICKS(20));
  }
}
bool storageIsErased() {
  const esp_partition_t *part=esp_partition_find_first(ESP_PARTITION_TYPE_DATA,ESP_PARTITION_SUBTYPE_DATA_SPIFFS,"spiffs");
  if(!part) return false;
  uint8_t block[512];
  for(size_t offset=0;offset<part->size;offset+=sizeof(block)) {
    if(esp_partition_read(part,offset,block,sizeof(block))!=ESP_OK) return false;
    for(uint8_t byte : block) if(byte!=0xff) return false;
  }
  return true;
}
void setup() {
  Serial.begin(115200); serialBuffer.reserve(4096); pinMode(FAN_PIN,OUTPUT);
  for(auto &m : motors) for(int i=0;i<4;i++) { digitalWrite(m.pins[i],HIGH); pinMode(m.pins[i],OUTPUT); }
  safeStop(); pinMode(UV_PIN,INPUT); analogReadResolution(12); analogSetPinAttenuation(UV_PIN,ADC_11db);
  for(auto &s : dht) s.begin();
  Preferences prefs; prefs.begin("filamentbox",false);
  bootNumber=prefs.getUInt("boot",0)+1; prefs.putUInt("boot",bootNumber); prefs.end();
  char nonce[33]; snprintf(nonce,sizeof(nonce),"%08lx%08lx%08lx%08lx",(unsigned long)esp_random(),(unsigned long)esp_random(),(unsigned long)esp_random(),(unsigned long)esp_random()); bootNonce=nonce;
  fsMutex=xSemaphoreCreateMutex(); cloudCommands=xQueueCreate(1,sizeof(Control)); latestTelemetry=xQueueCreate(1,sizeof(Packet));
  networkLogs=xQueueCreate(8,sizeof(NetworkLog));
  if(!fsMutex || !cloudCommands || !latestTelemetry || !networkLogs) { safeStop(); while(true) delay(1000); }
  fsReady=LittleFS.begin(false); // never erase pending data after a mount failure
  // Only an entirely erased partition may be initialized automatically.
  if(!fsReady && storageIsErased() && LittleFS.format()) fsReady=LittleFS.begin(false);
  if(!fsReady) Serial.println("[STORAGE] LittleFS unavailable; initialize once as documented.");
  lastSensor=millis()-SENSOR_MS+2500;
  if(xTaskCreate(networkTask,"network",16384,nullptr,1,nullptr)!=pdPASS) Serial.println("[NETWORK] Worker unavailable; USB remains active.");
}
void loop() {
  serialTask(); controlTask(); motorTask(); uvTask(); networkLogTask();
  if(!motors[0].remaining && !motors[1].remaining && millis()-lastSensor>=SENSOR_MS) { lastSensor=millis(); sampleSensors(); }
  delay(1);
}
