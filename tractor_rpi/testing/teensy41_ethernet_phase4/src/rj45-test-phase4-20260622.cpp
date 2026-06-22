// teensy41_ethernet_phase4
// Phase 4: SD logging + Ethernet web page + browser time sync
// Fix: dynamic homepage does NOT send Content-Length.
// CSV downloads still use Content-Length.

#include <Arduino.h>
#include <NativeEthernet.h>
#include <NativeEthernetUdp.h>
#include <SD.h>
#include <TimeLib.h>

// using namespace qindesign::network;

// -------------------------------------------------------------------
// Time config

#define TZ_OFFSET_HOURS (-4)
#define TZ_OFFSET_SEC (TZ_OFFSET_HOURS * 3600L)
#define TZ_LABEL "EDT"

bool rtcSet = false;
char rtcTimeStr[32] = "Not set";

time_t localNow() {
  return now() + TZ_OFFSET_SEC;
}

void updateRTCString() {
  if (timeStatus() != timeNotSet && year() >= 2025 && year() <= 2034) {
    time_t lt = localNow();
    tmElements_t tm;
    breakTime(lt, tm);

    snprintf(rtcTimeStr, sizeof(rtcTimeStr),
             "%04d-%02d-%02d %02d:%02d:%02d %s",
             tmYearToCalendar(tm.Year), tm.Month, tm.Day,
             tm.Hour, tm.Minute, tm.Second, TZ_LABEL);

    rtcSet = true;
  } else {
    strcpy(rtcTimeStr, "Not set");
    rtcSet = false;
  }
}

void setTimeFromUnix(unsigned long t) {
  if (t < 1735689600UL || t > 2051222400UL) {
    Serial.print("Time: rejected out-of-range timestamp: ");
    Serial.println(t);
    return;
  }

  setTime(t);
  updateRTCString();

  Serial.print("Time set: ");
  Serial.println(rtcTimeStr);
}

// -------------------------------------------------------------------
// NTP

bool syncNTP() {
  EthernetUDP udp;

  if (!udp.begin(2390)) return false;

  IPAddress ntpServer(216, 239, 35, 4);  // time4.google.com
  const unsigned long SEVENTY_YEARS = 2208988800UL;

  byte buf[48];
  memset(buf, 0, sizeof(buf));

  buf[0] = 0b11100011;
  buf[1] = 0;
  buf[2] = 6;
  buf[3] = 0xEC;
  buf[12] = 49;
  buf[13] = 0x4E;
  buf[14] = 49;
  buf[15] = 52;

  udp.beginPacket(ntpServer, 123);
  udp.write(buf, 48);
  udp.endPacket();

  unsigned long start = millis();

  while (millis() - start < 5000) {
    if (udp.parsePacket()) {
      udp.read(buf, 48);

      bool modeOk = ((buf[0] & 0x07) == 4);
      bool stratumOk = (buf[1] != 0);
      bool tsNonZero = (buf[40] || buf[41] || buf[42] || buf[43]);

      if (!modeOk || !stratumOk || !tsNonZero) {
        Serial.println("NTP: invalid response (bad mode/stratum/timestamp)");
        udp.stop();
        return false;
      }

      unsigned long hi = word(buf[40], buf[41]);
      unsigned long lo = word(buf[42], buf[43]);
      unsigned long t = ((hi << 16) | lo) - SEVENTY_YEARS;

      udp.stop();
      setTimeFromUnix(t);
      return true;
    }
  }

  udp.stop();
  return false;
}

// -------------------------------------------------------------------
// SD logging

File logFile;
char logFilename[26];
unsigned long rowCount = 0;
bool sdReady = false;

const unsigned long LOG_INTERVAL_MS = 100;
unsigned long lastLogTime = 0;

// Simulated robot data
float sim_steering = 512.0;
float sim_steering_dir = 1.0;
int sim_bucket = 5;
int sim_gps = 3;
bool sim_estop = false;

void updateSimData() {
  sim_steering += sim_steering_dir * 2.5;

  if (sim_steering > 712) sim_steering_dir = -1.0;
  if (sim_steering < 312) sim_steering_dir = 1.0;

  static unsigned long lastBucket = 0;
  if (millis() - lastBucket > 3000) {
    sim_bucket = (sim_bucket == 4) ? 5 : (sim_bucket == 5) ? 6 : 4;
    lastBucket = millis();
  }
}

void openLogFile() {
  updateRTCString();

  if (rtcSet) {
    tmElements_t tm;
    breakTime(localNow(), tm);

    snprintf(logFilename, sizeof(logFilename),
             "%04d%02d%02d_%02d%02d%02d.csv",
             tmYearToCalendar(tm.Year), tm.Month, tm.Day,
             tm.Hour, tm.Minute, tm.Second);
  } else {
    for (int i = 1; i <= 999; i++) {
      snprintf(logFilename, sizeof(logFilename), "log%03d.csv", i);
      if (!SD.exists(logFilename)) break;
    }
  }

  logFile = SD.open(logFilename, FILE_WRITE);

  if (logFile) {
    logFile.println("timestamp_ms,steering_val,transmission_bucket,gps_status,estop,uptime_s");
    logFile.flush();

    Serial.print("Logging to: ");
    Serial.println(logFilename);
  } else {
    Serial.println("ERROR: Could not open log file");
    sdReady = false;
  }
}

void setupSD() {
  Serial.println("Initializing SD card...");

  if (!SD.begin(BUILTIN_SDCARD)) {
    Serial.println("ERROR: SD card init failed. Check card is inserted.");
    return;
  }

  Serial.println("SD card ready.");
  sdReady = true;
  openLogFile();
}

void logRow() {
  if (!sdReady || !logFile) return;

  unsigned long nowMs = millis();

  if (nowMs - lastLogTime < LOG_INTERVAL_MS) return;
  lastLogTime = nowMs;

  updateSimData();

  logFile.print(nowMs);
  logFile.print(',');
  logFile.print(sim_steering, 1);
  logFile.print(',');
  logFile.print(sim_bucket);
  logFile.print(',');
  logFile.print(sim_gps);
  logFile.print(',');
  logFile.print(sim_estop ? 1 : 0);
  logFile.print(',');
  logFile.println(nowMs / 1000.0, 1);

  rowCount++;

  if (rowCount % 50 == 0) {
    logFile.flush();
    Serial.print("Flushed — rows: ");
    Serial.println(rowCount);
  }
}

void deleteAllFiles() {
  Serial.println("Deleting all CSV files...");

  if (logFile) logFile.close();

  File root = SD.open("/");

  while (true) {
    File entry = root.openNextFile();
    if (!entry) break;

    if (!entry.isDirectory()) {
      String name = entry.name();
      String upper = name;
      upper.toUpperCase();

      entry.close();

      if (upper.endsWith(".CSV")) {
        SD.remove(name.c_str());
        Serial.print("Deleted: ");
        Serial.println(name);
      }
    } else {
      entry.close();
    }
  }

  root.close();

  rowCount = 0;
  openLogFile();
}

// -------------------------------------------------------------------
// HTTP server
byte mac[] = { 0x04, 0xE9, 0xE5, 0x12, 0x34, 0x70 };
EthernetServer server(80);

void readRequestLine(EthernetClient& client, char* buf, int maxLen) {
  int i = 0;
  unsigned long t = millis();

  while (client.connected() && millis() - t < 2000) {
    if (client.available()) {
      char c = client.read();
      t = millis();

      if (c == '\n') break;
      if (c != '\r' && i < maxLen - 1) {
        buf[i++] = c;
      }
    }
  }

  buf[i] = '\0';
}

void drainHeaders(EthernetClient& client) {
  int newlines = 0;
  unsigned long t = millis();

  while (client.connected() && millis() - t < 2000) {
    if (client.available()) {
      char c = client.read();
      t = millis();

      if (c == '\n') {
        if (++newlines >= 2) break;
      } else if (c != '\r') {
        newlines = 0;
      }
    }
  }
}

void sendOkHeaders(EthernetClient& client,
                   const char* ct,
                   long len = -1,
                   bool attach = false,
                   const char* fn = nullptr) {
  client.print("HTTP/1.1 200 OK\r\n");
  client.print("Content-Type: ");
  client.print(ct);
  client.print("\r\n");

  if (len >= 0) {
    client.print("Content-Length: ");
    client.print(len);
    client.print("\r\n");
  }

  if (attach && fn) {
    client.print("Content-Disposition: attachment; filename=\"");
    client.print(fn);
    client.print("\"\r\n");
  }

  client.print("Accept-Ranges: none\r\n");
  client.print("Connection: close\r\n\r\n");
}

void sendRedirectHome(EthernetClient& client) {
  client.print("HTTP/1.1 303 See Other\r\n");
  client.print("Location: /\r\n");
  client.print("Connection: close\r\n\r\n");
  client.flush();
  client.stop();
}

// -------------------------------------------------------------------
// File download

void serveFileDownload(EthernetClient& client, const char* filename) {
  bool isActive = (strcmp(filename, logFilename) == 0);

  if (isActive && logFile) {
    logFile.flush();
    logFile.close();
  }

  File f = SD.open(filename);

  if (!f) {
    client.print("HTTP/1.1 404 Not Found\r\n");
    client.print("Connection: close\r\n\r\n");
    client.print("File not found.");
    client.flush();
    client.stop();

    if (isActive) {
      logFile = SD.open(logFilename, FILE_WRITE);
    }

    return;
  }

  unsigned long sz = f.size();

  sendOkHeaders(client, "application/octet-stream", sz, true, filename);

  uint8_t buf[512];

  while (f.available() && client.connected()) {
    int n = f.read(buf, sizeof(buf));
    int sent = 0;

    while (sent < n && client.connected()) {
      int s = client.write(buf + sent, n - sent);
      if (s > 0) sent += s;
    }
  }

  f.close();
  client.flush();
  client.stop();

  Serial.print("Downloaded: ");
  Serial.print(filename);
  Serial.print(" (");
  Serial.print(sz);
  Serial.println(" B)");

  if (isActive) {
    logFile = SD.open(logFilename, FILE_WRITE);

    if (!logFile) {
      Serial.println("ERROR: Could not reopen log");
      sdReady = false;
    }
  }
}

// -------------------------------------------------------------------
// Main page

void serveMainPage(EthernetClient& client) {
  IPAddress ip = Ethernet.localIP();

  char ipStr[24];
  snprintf(ipStr, sizeof(ipStr), "%u.%u.%u.%u", ip[0], ip[1], ip[2], ip[3]);

  updateRTCString();

  int fileCount = 0;
  long totalBytes = 0;

  // IMPORTANT:
  // Dynamic HTML page: no Content-Length.
  // This avoids broken browser/curl behavior if the generated page length changes.
  sendOkHeaders(client, "text/html", -1);

  client.print(
    "<!DOCTYPE html><html><head><title>Teensy Logger</title>"
    "<style>"
    "body{font-family:sans-serif;background:#1a1a2e;color:#e0e0e0;margin:0;padding:24px;}"
    "h1{color:#7ec8e3;margin:0 0 4px;}"
    ".sub{color:#777;font-size:.85em;margin-bottom:20px;}"
    ".card{background:#2d2d2d;border-radius:10px;padding:20px 24px;margin-bottom:18px;box-shadow:0 2px 10px rgba(0,0,0,.4);}"
    "h2{color:#aaa;font-size:.95em;font-weight:normal;margin:0 0 12px;text-transform:uppercase;letter-spacing:.05em;}"
    "table{width:100%;border-collapse:collapse;}"
    "th{text-align:left;color:#777;padding:5px 8px;border-bottom:2px solid #3a3a3a;font-weight:normal;font-size:.85em;text-transform:uppercase;}"
    "td{padding:9px 8px;border-bottom:1px solid #333;vertical-align:middle;}"
    "td.val{font-weight:bold;color:#a8d8a8;}"
    "td.num{color:#aaa;font-size:.9em;}"
    "td.empty{color:#555;text-align:center;padding:20px;}"
    ".rtc-ok{color:#a8d8a8 !important;}"
    ".rtc-bad{color:#e87070 !important;}"
    ".btn{background:#2a5a3a;color:#c8e8c8;padding:4px 14px;border-radius:5px;text-decoration:none;font-size:.85em;white-space:nowrap;}"
    ".btn:hover{background:#3a7a4a;}"
    ".del{background:#5a2a2a;color:#e8c8c8;}"
    ".del:hover{background:#7a3a3a;}"
    ".warn{background:#5a3a00;color:#f0c060;margin-top:14px;display:inline-block;}"
    ".tag{background:#1a3a5a;color:#7ec8e3;font-size:.75em;padding:2px 7px;border-radius:4px;margin-left:6px;}"
    "</style></head><body>"
  );

  client.print("<h1>Teensy 4.1 Logger</h1>");
  client.print("<div class='sub'>Phase 4 &mdash; SD + Time Sync &nbsp;&bull;&nbsp; ");
  client.print("<a href='/' style='color:#7ec8e3;'>Refresh</a></div>");

  client.print("<div class='card'><h2>Status</h2><table>");

  client.print("<tr><td>IP Address</td><td class='val'>");
  client.print(ipStr);
  client.print("</td></tr>");

  client.print("<tr><td>Active Log</td><td class='val'>");
  client.print(sdReady ? logFilename : "SD ERROR");
  client.print("</td></tr>");

  client.print("<tr><td>Rows Written</td><td class='val'>");
  client.print(rowCount);
  client.print("</td></tr>");

  client.print("<tr><td>Log Rate</td><td class='val'>10 Hz</td></tr>");

  // Count files before printing final total row
  if (sdReady) {
    File root = SD.open("/");

    while (true) {
      File entry = root.openNextFile();
      if (!entry) break;

      if (!entry.isDirectory()) {
        String name = entry.name();
        String upper = name;
        upper.toUpperCase();

        if (upper.endsWith(".CSV")) {
          fileCount++;
          totalBytes += entry.size();
        }
      }

      entry.close();
    }

    root.close();
  }

  client.print("<tr><td>Files on SD</td><td class='val'>");
  client.print(fileCount);
  client.print(" (");
  client.print(totalBytes);
  client.print(" B)</td></tr>");

  client.print("<tr><td>Uptime</td><td class='val'>");
  client.print(millis() / 1000);
  client.print(" s</td></tr>");

  client.print("<tr><td>Date / Time</td><td class='");
  client.print(rtcSet ? "val rtc-ok" : "val rtc-bad");
  client.print("'>");
  client.print(rtcTimeStr);
  client.print("</td></tr>");

  client.print("</table></div>");

  client.print("<div class='card'><h2>Log Files</h2>");
  client.print("<table><tr><th>File</th><th>Size</th><th></th><th></th></tr>");

  if (!sdReady || fileCount == 0) {
    client.print("<tr><td colspan='4' class='empty'>No CSV files on SD card</td></tr>");
  } else {
    File root = SD.open("/");

    while (true) {
      File entry = root.openNextFile();
      if (!entry) break;

      if (!entry.isDirectory()) {
        String name = entry.name();
        String upper = name;
        upper.toUpperCase();

        if (upper.endsWith(".CSV")) {
          bool active = (strcmp(entry.name(), logFilename) == 0);
          unsigned long sz = entry.size();

          client.print("<tr><td>");
          client.print(name);
          if (active) client.print("<span class='tag'>logging</span>");
          client.print("</td><td class='num'>");
          client.print(sz);
          client.print(" B</td><td>");

          client.print("<a class='btn' href='/");
          client.print(name);
          client.print("' download='");
          client.print(name);
          client.print("'>Download</a></td><td>");

          if (!active) {
            client.print("<a class='btn del' href='/?delete=");
            client.print(name);
            client.print("' onclick=\"return confirm('Delete ");
            client.print(name);
            client.print("?')\">Delete</a>");
          }

          client.print("</td></tr>");
        }
      }

      entry.close();
    }

    root.close();
  }

  client.print("</table>");

  client.print("<a class='btn del warn' href='/?deleteall' ");
  client.print("onclick=\"return confirm('Delete ALL CSV files?')\">");
  client.print("Delete All Files</a>");

  client.print("</div>");

  // Browser time sync if NTP failed
  client.print("<script>");
  client.print("const rtcSet=");
  client.print(rtcSet ? "true" : "false");
  client.print(";");
  client.print("if(!rtcSet){");
  client.print("fetch('/?settime='+Math.floor(Date.now()/1000)).then(()=>setTimeout(()=>location.reload(),500));");
  client.print("}");
  client.print("</script>");

  client.print("</body></html>");

  client.flush();
  delay(2);
  client.stop();
}

// -------------------------------------------------------------------
// Request router

void handleWeb() {
  EthernetClient client = server.accept();
  if (!client) return;

  char reqLine[128] = {0};
  readRequestLine(client, reqLine, sizeof(reqLine));
  drainHeaders(client);

  char path[64] = "/";

  if (strncmp(reqLine, "GET ", 4) == 0) {
    char* start = reqLine + 4;
    char* end = strchr(start, ' ');

    if (end) {
      int len = min((int)(end - start), 63);
      strncpy(path, start, len);
      path[len] = '\0';
    }
  }

  Serial.print("REQ: ");
  Serial.println(path);

  char* stParam = strstr(path, "?settime=");
  if (stParam) {
    unsigned long t = strtoul(stParam + 9, nullptr, 10);
    setTimeFromUnix(t);

    client.print("HTTP/1.1 200 OK\r\n");
    client.print("Content-Length: 0\r\n");
    client.print("Connection: close\r\n\r\n");
    client.flush();
    client.stop();
    return;
  }

  if (strstr(path, "?deleteall")) {
    deleteAllFiles();
    sendRedirectHome(client);
    return;
  }

  char* del = strstr(path, "?delete=");
  if (del) {
    char* fname = del + 8;

    if (sdReady &&
        strlen(fname) > 0 &&
        strcmp(fname, logFilename) != 0 &&
        SD.exists(fname)) {
      SD.remove(fname);
      Serial.print("Deleted: ");
      Serial.println(fname);
    }

    sendRedirectHome(client);
    return;
  }

  if (strlen(path) > 1) {
    char* fname = path + 1;
    char* dot = strrchr(fname, '.');

    if (dot && strcasecmp(dot, ".csv") == 0) {
      serveFileDownload(client, fname);
      return;
    }
  }

  serveMainPage(client);
}

// -------------------------------------------------------------------
// setup / loop

void setup() {
  Serial.begin(115200);
  while (!Serial && millis() < 4000);

  Serial.println("=== Teensy 4.1 Ethernet Phase 4 ===");
  Serial.println("Time: not set (waiting for NTP or browser sync)");

  setupSD();

  // Serial.println("Starting Ethernet (DHCP)...");
  // Ethernet.begin();

  // if (!Ethernet.waitForLocalIP(15000)) {
  //   Serial.println("WARNING: No DHCP — web server unavailable. SD logging continues.");
  // } else {
  //   IPAddress ip = Ethernet.localIP();

  //   Serial.print("IP Address: ");
  //   Serial.println(ip);

  //   Serial.println("Attempting NTP sync (time4.google.com)...");

  //   if (syncNTP()) {
  //     Serial.print("NTP OK: ");
  //     Serial.println(rtcTimeStr);
  //   } else {
  //     Serial.println("NTP failed — browser will sync time on first page load.");
  //   }

  //   server.begin();

  //   Serial.print("Browse to: http://");
  //   Serial.println(ip);

  Serial.println("Starting Ethernet (DHCP)...");

  if (Ethernet.begin(mac, 15000, 4000) == 0) {
    Serial.println("WARNING: No DHCP — web server unavailable. SD logging continues.");
  } else {
    IPAddress ip = Ethernet.localIP();

    Serial.print("IP Address: ");
    Serial.println(ip);

    Serial.println("Attempting NTP sync (time4.google.com)...");

    if (syncNTP()) {
      Serial.print("NTP OK: ");
      Serial.println(rtcTimeStr);
    } else {
      Serial.println("NTP failed — browser will sync time on first page load.");
    }

    server.begin();

    Serial.print("Browse to: http://");
    Serial.println(ip);
  }

  Serial.println("====================================");
}

void loop() {
  logRow();
  handleWeb();
}