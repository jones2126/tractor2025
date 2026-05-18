// teensy41_ethernet_phase3
// Phase 3: Web file listing, CSV download, and delete
//
// Builds on Phase 2: SD logging at 10 Hz continues unchanged.
// Adds:
//   GET /              → status + list of all CSV files on SD card
//   GET /log001.csv    → download that file to browser
//   GET /?delete=X     → delete file X, redirect back to /
//
// Hardware: Teensy 4.1 with Ethernet add-on board (RJ45), built-in SD slot

#include <Arduino.h>
#include <SD.h>
#include <QNEthernet.h>

using namespace qindesign::network;

// -------------------------------------------------------------------
// SD logging (unchanged from Phase 2)
File logFile;
char logFilename[16];
unsigned long rowCount = 0;
bool sdReady = false;

const unsigned long LOG_INTERVAL_MS = 100;  // 10 Hz
unsigned long lastLogTime = 0;

// Simulated robot data fields
float sim_steering     = 512.0;
float sim_steering_dir = 1.0;
int   sim_bucket       = 5;
int   sim_gps          = 3;
bool  sim_estop        = false;

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

void openNextLogFile() {
    for (int i = 1; i <= 999; i++) {
        snprintf(logFilename, sizeof(logFilename), "log%03d.csv", i);
        if (!SD.exists(logFilename)) {
            logFile = SD.open(logFilename, FILE_WRITE);
            if (logFile) {
                logFile.println("timestamp_ms,steering_val,transmission_bucket,"
                                "gps_status,estop,uptime_s");
                logFile.flush();
                Serial.print("Logging to: ");
                Serial.println(logFilename);
            }
            return;
        }
    }
    Serial.println("ERROR: All log slots full (log001-log999 exist)");
}

void setupSD() {
    Serial.println("Initializing SD card...");
    if (!SD.begin(BUILTIN_SDCARD)) {
        Serial.println("ERROR: SD card init failed. Check card is inserted.");
        return;
    }
    Serial.println("SD card ready.");
    sdReady = true;
    openNextLogFile();
}

void logRow() {
    if (!sdReady || !logFile) return;
    unsigned long now = millis();
    if (now - lastLogTime < LOG_INTERVAL_MS) return;
    lastLogTime = now;

    updateSimData();

    logFile.print(now);                   logFile.print(',');
    logFile.print(sim_steering, 1);       logFile.print(',');
    logFile.print(sim_bucket);            logFile.print(',');
    logFile.print(sim_gps);               logFile.print(',');
    logFile.print(sim_estop ? 1 : 0);    logFile.print(',');
    logFile.println(now / 1000.0, 1);

    rowCount++;
    if (rowCount % 50 == 0) {
        logFile.flush();
        Serial.print("Flushed — rows: ");
        Serial.println(rowCount);
    }
}

// -------------------------------------------------------------------
// Ethernet / HTTP
EthernetServer server(80);

// Read only the first line of the HTTP request, discard the rest
void readRequestLine(EthernetClient& client, char* buf, int maxLen) {
    int i = 0;
    unsigned long t = millis();
    while (client.connected() && millis() - t < 2000) {
        if (client.available()) {
            char c = client.read();
            t = millis();
            if (c == '\n') break;
            if (c != '\r' && i < maxLen - 1) buf[i++] = c;
        }
    }
    buf[i] = '\0';
}

// Consume remaining HTTP headers (stop at blank line)
void drainHeaders(EthernetClient& client) {
    int newlines = 0;
    unsigned long t = millis();
    while (client.connected() && millis() - t < 2000) {
        if (client.available()) {
            char c = client.read();
            t = millis();
            if      (c == '\n') { if (++newlines >= 2) break; }
            else if (c != '\r') newlines = 0;
        }
    }
}

void sendOkHeaders(EthernetClient& client, const char* contentType,
                   long contentLen = -1,
                   bool attachment = false, const char* filename = nullptr) {
    client.println("HTTP/1.1 200 OK");
    client.print("Content-Type: "); client.println(contentType);
    if (contentLen >= 0) {
        client.print("Content-Length: "); client.println(contentLen);
    }
    if (attachment && filename) {
        client.print("Content-Disposition: attachment; filename=\"");
        client.print(filename); client.println("\"");
    }
    client.println("Connection: close");
    client.println();
}

// -------------------------------------------------------------------
// Route: GET /filename.csv  →  stream file to browser
void serveFileDownload(EthernetClient& client, const char* filename) {
    // Flush active log before reading it so data is complete
    if (strcmp(filename, logFilename) == 0 && logFile) logFile.flush();

    File f = SD.open(filename);
    if (!f) {
        client.println("HTTP/1.1 404 Not Found");
        client.println("Connection: close\r\n");
        client.println("File not found.");
        client.stop();
        return;
    }

    sendOkHeaders(client, "text/csv", f.size(), true, filename);

    uint8_t buf[512];
    while (f.available()) {
        int n = f.read(buf, sizeof(buf));
        client.write(buf, n);
    }
    f.close();
    client.flush();
    client.stop();

    Serial.print("Downloaded: "); Serial.print(filename);
    Serial.print("  ("); Serial.print(f.size()); Serial.println(" B)");
}

// -------------------------------------------------------------------
// Route: GET /  →  status card + file table
void serveMainPage(EthernetClient& client) {
    IPAddress ip = Ethernet.localIP();
    String ipStr = String(ip[0]) + "." + String(ip[1]) + "." +
                   String(ip[2]) + "." + String(ip[3]);

    // Build file rows by scanning SD root
    String fileRows = "";
    int    fileCount = 0;
    long   totalBytes = 0;

    if (sdReady) {
        File root = SD.open("/");
        while (true) {
            File entry = root.openNextFile();
            if (!entry) break;
            if (!entry.isDirectory()) {
                String name = entry.name();
                name.toUpperCase();
                bool isCSV = name.endsWith(".CSV");
                name = entry.name();  // restore original case
                if (isCSV) {
                    long sz = entry.size();
                    totalBytes += sz;
                    fileCount++;
                    bool active = (strcmp(entry.name(), logFilename) == 0);

                    fileRows += "<tr><td>" + name;
                    if (active) fileRows += " <span class='tag'>logging</span>";
                    fileRows += "</td><td class='num'>" + String(sz) + " B</td>";
                    fileRows += "<td><a class='btn' href='/" + name + "'>Download</a></td><td>";
                    if (!active) {
                        fileRows += "<a class='btn del' href='/?delete=" + name +
                                    "' onclick=\"return confirm('Delete " + name + "?')\">Delete</a>";
                    }
                    fileRows += "</td></tr>";
                }
            }
            entry.close();
        }
        root.close();
    }

    if (fileCount == 0) {
        fileRows = "<tr><td colspan='4' class='empty'>No CSV files found on SD card</td></tr>";
    }

    String body =
        "<!DOCTYPE html><html><head><title>Teensy Logger</title>"
        "<meta http-equiv='refresh' content='10'>"
        "<style>"
        "body{font-family:sans-serif;background:#1a1a2e;color:#e0e0e0;margin:0;padding:24px;}"
        "h1{color:#7ec8e3;margin:0 0 4px;}"
        ".sub{color:#666;font-size:.85em;margin-bottom:20px;}"
        ".card{background:#2d2d2d;border-radius:10px;padding:20px 24px;margin-bottom:18px;"
        "      box-shadow:0 2px 10px rgba(0,0,0,.4);}"
        "h2{color:#aaa;font-size:.95em;font-weight:normal;margin:0 0 12px;letter-spacing:.05em;"
        "   text-transform:uppercase;}"
        "table{width:100%;border-collapse:collapse;}"
        "th{text-align:left;color:#666;padding:5px 8px;border-bottom:2px solid #3a3a3a;"
        "   font-weight:normal;font-size:.85em;text-transform:uppercase;letter-spacing:.05em;}"
        "td{padding:9px 8px;border-bottom:1px solid #333;vertical-align:middle;}"
        "td.val{font-weight:bold;color:#a8d8a8;}"
        "td.num{color:#aaa;font-size:.9em;}"
        "td.empty{color:#555;text-align:center;padding:20px;}"
        ".btn{background:#2a5a3a;color:#c8e8c8;padding:4px 14px;border-radius:5px;"
        "     text-decoration:none;font-size:.85em;white-space:nowrap;}"
        ".btn:hover{background:#3a7a4a;}"
        ".del{background:#5a2a2a;color:#e8c8c8;}"
        ".del:hover{background:#7a3a3a;}"
        ".tag{background:#1a3a5a;color:#7ec8e3;font-size:.75em;padding:2px 7px;"
        "     border-radius:4px;margin-left:6px;vertical-align:middle;}"
        "</style></head><body>"

        "<h1>Teensy 4.1 Logger</h1>"
        "<div class='sub'>Phase 3 &mdash; SD File Download &nbsp;&bull;&nbsp; auto-refresh 10s</div>"

        "<div class='card'><h2>Status</h2><table>"
        "<tr><td>IP Address</td><td class='val'>"  + ipStr + "</td></tr>"
        "<tr><td>Active Log</td><td class='val'>"  + String(sdReady ? logFilename : "SD ERROR") + "</td></tr>"
        "<tr><td>Rows Written</td><td class='val'>" + String(rowCount) + "</td></tr>"
        "<tr><td>Log Rate</td><td class='val'>10 Hz</td></tr>"
        "<tr><td>Files on SD</td><td class='val'>" + String(fileCount) + " files &nbsp;("
                                                    + String(totalBytes) + " B total)</td></tr>"
        "<tr><td>Uptime</td><td class='val'>"      + String(millis() / 1000) + " s</td></tr>"
        "</table></div>"

        "<div class='card'><h2>Log Files</h2><table>"
        "<tr><th>File</th><th>Size</th><th></th><th></th></tr>"
        + fileRows +
        "</table></div>"

        "</body></html>";

    sendOkHeaders(client, "text/html", body.length());
    client.print(body);
    client.flush();
    client.stop();
}

// -------------------------------------------------------------------
void handleWeb() {
    EthernetClient client = server.accept();
    if (!client) return;

    char reqLine[128] = {0};
    readRequestLine(client, reqLine, sizeof(reqLine));
    drainHeaders(client);

    // Extract path from "GET /path HTTP/1.1"
    char path[64] = "/";
    if (strncmp(reqLine, "GET ", 4) == 0) {
        char* start = reqLine + 4;
        char* end   = strchr(start, ' ');
        if (end) {
            int len = (int)(end - start);
            if (len > 63) len = 63;
            strncpy(path, start, len);
            path[len] = '\0';
        }
    }

    // Route: /?delete=filename  →  delete file, redirect to /
    char* del = strstr(path, "?delete=");
    if (del) {
        char* fname = del + 8;
        if (sdReady && strlen(fname) > 0 && SD.exists(fname)) {
            SD.remove(fname);
            Serial.print("Deleted: "); Serial.println(fname);
        }
        client.println("HTTP/1.1 303 See Other");
        client.println("Location: /");
        client.println("Connection: close");
        client.println();
        client.stop();
        return;
    }

    // Route: /filename.csv  →  download
    if (strlen(path) > 1) {
        char* fname = path + 1;          // strip leading /
        char* dot   = strrchr(fname, '.');
        if (dot && (strcasecmp(dot, ".csv") == 0) && SD.exists(fname)) {
            serveFileDownload(client, fname);
            return;
        }
    }

    // Default: main page
    serveMainPage(client);
}

// -------------------------------------------------------------------
void setup() {
    Serial.begin(115200);
    while (!Serial && millis() < 4000);
    Serial.println("=== Teensy 4.1 Ethernet Phase 3 ===");

    setupSD();

    Serial.println("Starting Ethernet (DHCP)...");
    Ethernet.begin();
    if (!Ethernet.waitForLocalIP(15000)) {
        Serial.println("WARNING: No DHCP — web server unavailable.");
        Serial.println("SD logging will continue without Ethernet.");
    } else {
        IPAddress ip = Ethernet.localIP();
        Serial.print("IP Address: "); Serial.println(ip);
        server.begin();
        Serial.print("Browse to:  http://"); Serial.println(ip);
    }
    Serial.println("====================================");
}

void loop() {
    logRow();    // SD CSV logging at 10 Hz (non-blocking)
    handleWeb(); // HTTP: serve page or file if a client connects
}
