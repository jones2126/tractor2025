// teensy41_ethernet_phase4
// Adds to Phase 3:
//   - Teensy 4.1 built-in RTC for timekeeping (persists across power cycles with coin cell)
//   - NTP sync on startup via UDP (Google time server, no DNS needed)
//   - Browser auto-sync: page JS silently sends Unix timestamp if RTC not set
//   - Timestamped log filenames (20260519_143052.csv) when RTC is available
//   - Delete All button: wipes all CSV files, reopens fresh log
//
// Time priority: NTP (auto, internet) → browser JS (auto, local) → sequential filenames

#include <Arduino.h>
#include <SD.h>
#include <TimeLib.h>
#include <QNEthernet.h>

using namespace qindesign::network;

// -------------------------------------------------------------------
// Software time — no hardware RTC. Set from NTP or browser on each boot.
// TimeLib tracks time in software from the moment setTime() is called.
// Time is stored as UTC internally; localNow() applies the timezone offset.

// US Eastern: EDT = UTC-4 (Mar–Nov), EST = UTC-5 (Nov–Mar)
// Change TZ_OFFSET_HOURS to -5 in winter when EST is in effect
#define TZ_OFFSET_HOURS  (-4)
#define TZ_OFFSET_SEC    (TZ_OFFSET_HOURS * 3600L)
#define TZ_LABEL         "EDT"

time_t localNow() { return now() + TZ_OFFSET_SEC; }

bool rtcSet = false;
char rtcTimeStr[32] = "Not set";

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
    // Sanity: accept only 2025-01-01 to 2034-12-31
    // Rejects NTP underflow artifacts and obvious garbage
    if (t < 1735689600UL || t > 2051222400UL) {
        Serial.print("Time: rejected out-of-range timestamp: "); Serial.println(t);
        return;
    }
    setTime(t);   // TimeLib software clock — no hardware write
    updateRTCString();
    Serial.print("Time set: "); Serial.println(rtcTimeStr);
}

// -------------------------------------------------------------------
// NTP — manual UDP client using Google's NTP IP (no DNS required)
// 216.239.35.4 = time4.google.com

bool syncNTP() {
    EthernetUDP udp;
    if (!udp.begin(2390)) return false;

    IPAddress ntpServer(216, 239, 35, 4);
    const unsigned long SEVENTY_YEARS = 2208988800UL;

    byte buf[48];
    memset(buf, 0, 48);
    buf[0] = 0b11100011;   // LI=0, Version=4, Mode=3 (client)
    buf[1] = 0;
    buf[2] = 6;
    buf[3] = 0xEC;
    buf[12] = 49; buf[13] = 0x4E; buf[14] = 49; buf[15] = 52;

    udp.beginPacket(ntpServer, 123);
    udp.write(buf, 48);
    udp.endPacket();

    unsigned long start = millis();
    while (millis() - start < 5000) {
        if (udp.parsePacket()) {
            udp.read(buf, 48);

            // Validate: Mode bits (buf[0] & 0x07) must be 4 (server response)
            // Stratum (buf[1]) must be non-zero (0 = unspecified/invalid)
            // Transmit timestamp seconds (bytes 40-43) must be non-zero
            bool modeOk    = ((buf[0] & 0x07) == 4);
            bool stratumOk = (buf[1] != 0);
            bool tsNonZero = (buf[40] || buf[41] || buf[42] || buf[43]);

            if (!modeOk || !stratumOk || !tsNonZero) {
                Serial.println("NTP: invalid response (bad mode/stratum/timestamp)");
                udp.stop();
                return false;
            }

            unsigned long hi = word(buf[40], buf[41]);
            unsigned long lo = word(buf[42], buf[43]);
            unsigned long t  = (hi << 16 | lo) - SEVENTY_YEARS;
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

File  logFile;
char  logFilename[26];   // YYYYMMDD_HHMMSS.csv = 19 chars + null; 26 avoids compiler truncation warning
unsigned long rowCount   = 0;
bool          sdReady    = false;

const unsigned long LOG_INTERVAL_MS = 100;   // 10 Hz
unsigned long lastLogTime = 0;

// Simulated robot data fields — replace with real sensor reads later
float sim_steering = 512.0, sim_steering_dir = 1.0;
int   sim_bucket   = 5,     sim_gps          = 3;
bool  sim_estop    = false;

void updateSimData() {
    sim_steering += sim_steering_dir * 2.5;
    if (sim_steering > 712) sim_steering_dir = -1.0;
    if (sim_steering < 312) sim_steering_dir =  1.0;
    static unsigned long lastBucket = 0;
    if (millis() - lastBucket > 3000) {
        sim_bucket = (sim_bucket == 4) ? 5 : (sim_bucket == 5) ? 6 : 4;
        lastBucket = millis();
    }
}

// Open log file — timestamp name if RTC set, sequential if not
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
        logFile.println("timestamp_ms,steering_val,transmission_bucket,"
                        "gps_status,estop,uptime_s");
        logFile.flush();
        Serial.print("Logging to: "); Serial.println(logFilename);
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
    unsigned long now = millis();
    if (now - lastLogTime < LOG_INTERVAL_MS) return;
    lastLogTime = now;
    updateSimData();
    logFile.print(now);                logFile.print(',');
    logFile.print(sim_steering, 1);    logFile.print(',');
    logFile.print(sim_bucket);         logFile.print(',');
    logFile.print(sim_gps);            logFile.print(',');
    logFile.print(sim_estop ? 1 : 0);  logFile.print(',');
    logFile.println(now / 1000.0, 1);
    rowCount++;
    if (rowCount % 50 == 0) {
        logFile.flush();
        Serial.print("Flushed — rows: "); Serial.println(rowCount);
    }
}

// Delete all CSV files, reopen a fresh log
void deleteAllFiles() {
    Serial.println("Deleting all CSV files...");
    if (logFile) logFile.close();

    File root = SD.open("/");
    while (true) {
        File entry = root.openNextFile();
        if (!entry) break;
        if (!entry.isDirectory()) {
            String name = entry.name();
            String upper = name; upper.toUpperCase();
            entry.close();
            if (upper.endsWith(".CSV")) {
                SD.remove(name.c_str());
                Serial.print("Deleted: "); Serial.println(name);
            }
        } else {
            entry.close();
        }
    }
    root.close();

    rowCount = 0;
    openLogFile();   // Will use timestamp name if RTC is now set
}

// -------------------------------------------------------------------
// HTTP helpers (same as Phase 3)

EthernetServer server(80);

void readRequestLine(EthernetClient& client, char* buf, int maxLen) {
    int i = 0;
    unsigned long t = millis();
    while (client.connected() && millis() - t < 2000) {
        if (client.available()) {
            char c = client.read(); t = millis();
            if (c == '\n') break;
            if (c != '\r' && i < maxLen - 1) buf[i++] = c;
        }
    }
    buf[i] = '\0';
}

void drainHeaders(EthernetClient& client) {
    int newlines = 0;
    unsigned long t = millis();
    while (client.connected() && millis() - t < 2000) {
        if (client.available()) {
            char c = client.read(); t = millis();
            if      (c == '\n') { if (++newlines >= 2) break; }
            else if (c != '\r') newlines = 0;
        }
    }
}

void sendOkHeaders(EthernetClient& client, const char* ct, long len = -1,
                   bool attach = false, const char* fn = nullptr) {
    client.print("HTTP/1.1 200 OK\r\n");
    client.print("Content-Type: "); client.print(ct); client.print("\r\n");
    if (len >= 0) { client.print("Content-Length: "); client.print(len); client.print("\r\n"); }
    if (attach && fn) {
        client.print("Content-Disposition: attachment; filename=\"");
        client.print(fn); client.print("\"\r\n");
    }
    client.print("Accept-Ranges: none\r\n");
    client.print("Connection: close\r\n\r\n");
}

// -------------------------------------------------------------------
// Route: file download

void serveFileDownload(EthernetClient& client, const char* filename) {
    bool isActive = (strcmp(filename, logFilename) == 0);
    if (isActive && logFile) logFile.close();

    File f = SD.open(filename);
    if (!f) {
        client.print("HTTP/1.1 404 Not Found\r\nConnection: close\r\n\r\nFile not found.");
        client.stop();
        if (isActive) logFile = SD.open(logFilename, FILE_WRITE);
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
    f.close(); client.flush(); client.stop();
    Serial.print("Downloaded: "); Serial.print(filename);
    Serial.print("  ("); Serial.print(sz); Serial.println(" B)");
    if (isActive) {
        logFile = SD.open(logFilename, FILE_WRITE);
        if (!logFile) { Serial.println("ERROR: Could not reopen log"); sdReady = false; }
    }
}

// -------------------------------------------------------------------
// Route: main page

void serveMainPage(EthernetClient& client) {
    IPAddress ip = Ethernet.localIP();
    String ipStr = String(ip[0])+"."+String(ip[1])+"."+String(ip[2])+"."+String(ip[3]);

    updateRTCString();

    // Build file table
    String fileRows = "";
    int fileCount = 0; long totalBytes = 0;
    if (sdReady) {
        File root = SD.open("/");
        while (true) {
            File entry = root.openNextFile();
            if (!entry) break;
            if (!entry.isDirectory()) {
                String name = entry.name();
                String upper = name; upper.toUpperCase();
                bool isCSV = upper.endsWith(".CSV");
                if (isCSV) {
                    long sz = entry.size(); totalBytes += sz; fileCount++;
                    bool active = (strcmp(entry.name(), logFilename) == 0);
                    fileRows += "<tr><td>" + name;
                    if (active) fileRows += " <span class='tag'>logging</span>";
                    fileRows += "</td><td class='num'>" + String(sz) + " B</td>";
                    fileRows += "<td><a class='btn' href='/" + name + "' download='" + name + "'>Download</a></td>";
                    fileRows += "<td>";
                    if (!active) fileRows += "<a class='btn del' href='/?delete=" + name + "' onclick=\"return confirm('Delete " + name + "?')\">Delete</a>";
                    fileRows += "</td></tr>";
                }
            }
            entry.close();
        }
        root.close();
    }
    if (fileCount == 0)
        fileRows = "<tr><td colspan='4' class='empty'>No CSV files on SD card</td></tr>";

    String rtcClass = rtcSet ? "val rtc-ok" : "val rtc-bad";
    String rtcSetJS = rtcSet ? "true" : "false";

    String body =
        "<!DOCTYPE html><html><head><title>Teensy Logger</title>"
        "<style>"
        "body{font-family:sans-serif;background:#1a1a2e;color:#e0e0e0;margin:0;padding:24px;}"
        "h1{color:#7ec8e3;margin:0 0 4px;}"
        ".sub{color:#666;font-size:.85em;margin-bottom:20px;}"
        ".card{background:#2d2d2d;border-radius:10px;padding:20px 24px;margin-bottom:18px;box-shadow:0 2px 10px rgba(0,0,0,.4);}"
        "h2{color:#aaa;font-size:.95em;font-weight:normal;margin:0 0 12px;text-transform:uppercase;letter-spacing:.05em;}"
        "table{width:100%;border-collapse:collapse;}"
        "th{text-align:left;color:#666;padding:5px 8px;border-bottom:2px solid #3a3a3a;font-weight:normal;font-size:.85em;text-transform:uppercase;}"
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
        ".warn:hover{background:#7a5000;}"
        ".tag{background:#1a3a5a;color:#7ec8e3;font-size:.75em;padding:2px 7px;border-radius:4px;margin-left:6px;}"
        "</style></head><body>"

        "<h1>Teensy 4.1 Logger</h1>"
        "<div class='sub'>Phase 4 &mdash; SD + Time Sync"
        " &nbsp;&bull;&nbsp; <a href='/' style='color:#7ec8e3;'>Refresh</a></div>"

        "<div class='card'><h2>Status</h2><table>"
        "<tr><td>IP Address</td><td class='val'>"    + ipStr + "</td></tr>"
        "<tr><td>Active Log</td><td class='val'>"    + String(sdReady ? logFilename : "SD ERROR") + "</td></tr>"
        "<tr><td>Rows Written</td><td class='val'>"  + String(rowCount) + "</td></tr>"
        "<tr><td>Log Rate</td><td class='val'>10 Hz</td></tr>"
        "<tr><td>Files on SD</td><td class='val'>"   + String(fileCount) + " (" + String(totalBytes) + " B)</td></tr>"
        "<tr><td>Uptime</td><td class='val'>"        + String(millis() / 1000) + " s</td></tr>"
        "<tr><td>Date / Time</td><td class='"        + rtcClass + "'>" + String(rtcTimeStr) + "</td></tr>"
        "</table></div>"

        "<div class='card'><h2>Log Files</h2>"
        "<table><tr><th>File</th><th>Size</th><th></th><th></th></tr>"
        + fileRows +
        "</table>"
        "<a class='btn warn' href='/?deleteall'"
        " onclick=\"return confirm('Delete ALL CSV files and start a fresh log?')\">"
        "&#128465;&nbsp;Delete All Files</a>"
        "</div>"

        // JS: silently send browser time if RTC not set; runs on every page load
        "<script>"
        "var rtcSet=" + rtcSetJS + ";"
        "if(!rtcSet){"
        "  var t=Math.floor(Date.now()/1000);"
        "  fetch('/?settime='+t).catch(function(){});"
        "  console.log('Sent time to Teensy: '+t);"
        "}"
        "</script>"
        "</body></html>";

    sendOkHeaders(client, "text/html", body.length());
    client.print(body);
    client.flush(); client.stop();
}

// -------------------------------------------------------------------
// Request router

void handleWeb() {
    EthernetClient client = server.accept();
    if (!client) return;

    char reqLine[128] = {0};
    readRequestLine(client, reqLine, sizeof(reqLine));
    drainHeaders(client);

    // Extract path
    char path[64] = "/";
    if (strncmp(reqLine, "GET ", 4) == 0) {
        char* start = reqLine + 4;
        char* end   = strchr(start, ' ');
        if (end) {
            int len = min((int)(end - start), 63);
            strncpy(path, start, len);
            path[len] = '\0';
        }
    }
    Serial.print("REQ: "); Serial.println(path);

    // /?settime=UNIX_TIMESTAMP  — browser sends this silently on page load
    char* stParam = strstr(path, "?settime=");
    if (stParam) {
        unsigned long t = strtoul(stParam + 9, nullptr, 10);
        setTimeFromUnix(t);
        client.print("HTTP/1.1 200 OK\r\nContent-Length: 0\r\nConnection: close\r\n\r\n");
        client.flush(); client.stop();
        return;
    }

    // /?deleteall — wipe all CSV files, reopen fresh log
    if (strstr(path, "?deleteall")) {
        deleteAllFiles();
        client.print("HTTP/1.1 303 See Other\r\nLocation: /\r\nConnection: close\r\n\r\n");
        client.flush(); client.stop();
        return;
    }

    // /?delete=filename — delete one file
    char* del = strstr(path, "?delete=");
    if (del) {
        char* fname = del + 8;
        if (sdReady && strlen(fname) > 0 && SD.exists(fname)) {
            SD.remove(fname);
            Serial.print("Deleted: "); Serial.println(fname);
        }
        client.print("HTTP/1.1 303 See Other\r\nLocation: /\r\nConnection: close\r\n\r\n");
        client.flush(); client.stop();
        return;
    }

    // /filename.csv — download
    if (strlen(path) > 1) {
        char* fname = path + 1;
        char* dot   = strrchr(fname, '.');
        if (dot && strcasecmp(dot, ".csv") == 0) {
            serveFileDownload(client, fname);
            return;
        }
    }

    serveMainPage(client);
}

// -------------------------------------------------------------------
void setup() {
    Serial.begin(115200);
    while (!Serial && millis() < 4000);
    Serial.println("=== Teensy 4.1 Ethernet Phase 4 ===");

    // Time starts unknown — will be set by NTP or browser JS
    Serial.println("Time: not set (waiting for NTP or browser sync)");

    // SD — open log (sequential name until time is confirmed)
    setupSD();

    // Ethernet + NTP
    Serial.println("Starting Ethernet (DHCP)...");
    Ethernet.begin();
    if (!Ethernet.waitForLocalIP(15000)) {
        Serial.println("WARNING: No DHCP — web server unavailable. SD logging continues.");
    } else {
        IPAddress ip = Ethernet.localIP();
        Serial.print("IP Address: "); Serial.println(ip);

        // Attempt NTP (5 second timeout)
        Serial.println("Attempting NTP sync (time4.google.com)...");
        if (syncNTP()) {
            Serial.print("NTP OK: "); Serial.println(rtcTimeStr);
        } else {
            Serial.println("NTP failed — browser will sync time on first page load.");
        }

        server.begin();
        Serial.print("Browse to:  http://"); Serial.println(ip);
    }
    Serial.println("====================================");
}

void loop() {
    logRow();    // 10 Hz SD logging (non-blocking)
    handleWeb(); // HTTP server
}
