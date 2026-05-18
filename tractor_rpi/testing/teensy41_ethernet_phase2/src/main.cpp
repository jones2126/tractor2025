// teensy41_ethernet_phase2
// Phase 2: SD card CSV logging at 10 Hz + Ethernet status page
//
// Builds on Phase 1: DHCP + HTTP server still running.
// Adds: SD card init, auto-numbered log file, 10 Hz CSV rows.
// Phase 3 will add file listing and download to the web page.
//
// Hardware: Teensy 4.1 with Ethernet add-on board (RJ45)
//           Uses built-in SD card slot (no wiring needed)

#include <Arduino.h>
#include <SD.h>
#include <QNEthernet.h>

using namespace qindesign::network;

// -------------------------------------------------------------------
// SD logging
File logFile;
char logFilename[16];       // e.g. "log003.csv"
unsigned long rowCount = 0;
bool sdReady = false;

const unsigned long LOG_INTERVAL_MS = 100;  // 10 Hz
unsigned long lastLogTime = 0;

// -------------------------------------------------------------------
// Ethernet / HTTP
EthernetServer server(80);

// -------------------------------------------------------------------
// Sample data (simulated robot fields — replace with real reads later)
// Mimics what the tractor main loop would log:
//   steering pot, transmission bucket, GPS status, radio signal, e-stop
float sim_steering  = 512.0;
float sim_steering_dir = 1.0;
int   sim_bucket    = 5;
int   sim_gps       = 3;      // 3 = RTK fix
bool  sim_estop     = false;

void updateSimData() {
    // Gentle sine-wave sweep on steering (512 ± 200)
    sim_steering += sim_steering_dir * 2.5;
    if (sim_steering > 712) sim_steering_dir = -1.0;
    if (sim_steering < 312) sim_steering_dir =  1.0;

    // Slowly cycle transmission bucket 4-6 (neutral region)
    static unsigned long lastBucket = 0;
    if (millis() - lastBucket > 3000) {
        sim_bucket = (sim_bucket == 4) ? 5 : (sim_bucket == 5) ? 6 : 4;
        lastBucket = millis();
    }
}

// -------------------------------------------------------------------
// Find next available log file name: log001.csv, log002.csv, ...
void openNextLogFile() {
    for (int i = 1; i <= 999; i++) {
        snprintf(logFilename, sizeof(logFilename), "log%03d.csv", i);
        if (!SD.exists(logFilename)) {
            logFile = SD.open(logFilename, FILE_WRITE);
            if (logFile) {
                // Write CSV header
                logFile.println("timestamp_ms,steering_val,transmission_bucket,"
                                "gps_status,estop,uptime_s");
                logFile.flush();
                Serial.print("Logging to: ");
                Serial.println(logFilename);
            } else {
                Serial.print("ERROR: Could not open ");
                Serial.println(logFilename);
            }
            return;
        }
    }
    Serial.println("ERROR: No available log file slots (log001-log999 all exist)");
}

// -------------------------------------------------------------------
void setupSD() {
    Serial.println("Initializing SD card...");
    if (!SD.begin(BUILTIN_SDCARD)) {
        Serial.println("ERROR: SD card init failed. Check card is inserted.");
        sdReady = false;
        return;
    }
    Serial.println("SD card ready.");
    sdReady = true;
    openNextLogFile();
}

// -------------------------------------------------------------------
void logRow() {
    if (!sdReady || !logFile) return;

    unsigned long now = millis();
    if (now - lastLogTime < LOG_INTERVAL_MS) return;
    lastLogTime = now;

    updateSimData();

    // Write one CSV row
    logFile.print(now);                       logFile.print(',');
    logFile.print(sim_steering, 1);           logFile.print(',');
    logFile.print(sim_bucket);                logFile.print(',');
    logFile.print(sim_gps);                   logFile.print(',');
    logFile.print(sim_estop ? 1 : 0);         logFile.print(',');
    logFile.println(now / 1000.0, 1);

    rowCount++;

    // Flush every 50 rows (~5 seconds) to protect against power loss
    if (rowCount % 50 == 0) {
        logFile.flush();
        Serial.print("Flushed — rows written: ");
        Serial.println(rowCount);
    }
}

// -------------------------------------------------------------------
void serveStatusPage(EthernetClient& client) {
    IPAddress ip = Ethernet.localIP();
    String ipStr = String(ip[0]) + "." + String(ip[1]) + "." +
                   String(ip[2]) + "." + String(ip[3]);

    String sdStatus  = sdReady ? String(logFilename) : "SD ERROR";
    String rowStr    = String(rowCount);
    String uptimeStr = String(millis() / 1000) + "s";
    String rateStr   = "10 Hz";

    String body =
        "<!DOCTYPE html><html><head><title>Teensy 4.1 Logger</title>"
        "<meta http-equiv='refresh' content='5'>"   // auto-refresh every 5s
        "<style>"
        "  body{font-family:sans-serif;background:#1e1e1e;color:#e0e0e0;"
        "       display:flex;justify-content:center;align-items:center;"
        "       min-height:100vh;margin:0;}"
        "  .card{background:#2d2d2d;border-radius:12px;padding:40px 60px;"
        "        box-shadow:0 4px 20px rgba(0,0,0,0.5);min-width:340px;}"
        "  h1{color:#7ec8e3;margin-bottom:4px;}"
        "  .sub{color:#888;font-size:0.85em;margin-bottom:24px;}"
        "  table{width:100%;border-collapse:collapse;}"
        "  td{padding:8px 4px;border-bottom:1px solid #3a3a3a;}"
        "  td:first-child{color:#aaa;width:50%;}"
        "  td:last-child{font-weight:bold;color:#a8d8a8;}"
        "  .ok{color:#a8d8a8;} .err{color:#e87070;}"
        "</style></head><body>"
        "<div class='card'>"
        "<h1>Teensy 4.1 Logger</h1>"
        "<div class='sub'>Phase 2 — SD Card Logging &nbsp;|&nbsp; auto-refresh 5s</div>"
        "<table>"
        "<tr><td>IP Address</td><td>" + ipStr + "</td></tr>"
        "<tr><td>Log File</td><td class='" + (sdReady ? "ok" : "err") + "'>" + sdStatus + "</td></tr>"
        "<tr><td>Rows Written</td><td>" + rowStr + "</td></tr>"
        "<tr><td>Log Rate</td><td>" + rateStr + "</td></tr>"
        "<tr><td>Uptime</td><td>" + uptimeStr + "</td></tr>"
        "</table>"
        "</div></body></html>";

    client.println("HTTP/1.1 200 OK");
    client.println("Content-Type: text/html");
    client.print("Content-Length: "); client.println(body.length());
    client.println("Connection: close");
    client.println();
    client.print(body);
    client.flush();
    client.stop();
}

// -------------------------------------------------------------------
void handleWeb() {
    EthernetClient client = server.accept();
    if (!client) return;

    unsigned long timeout = millis();
    while (client.connected() && millis() - timeout < 1000) {
        if (client.available()) { client.read(); timeout = millis(); }
    }

    serveStatusPage(client);
    Serial.print("Web page served — rows=");
    Serial.println(rowCount);
}

// -------------------------------------------------------------------
void setup() {
    Serial.begin(115200);
    while (!Serial && millis() < 4000);

    Serial.println("=== Teensy 4.1 Ethernet Phase 2 ===");

    // SD first — independent of Ethernet
    setupSD();

    // Ethernet
    Serial.println("Starting Ethernet (DHCP)...");
    Ethernet.begin();
    if (!Ethernet.waitForLocalIP(15000)) {
        Serial.println("WARNING: No DHCP lease — web server unavailable.");
        Serial.println("SD logging will continue without Ethernet.");
    } else {
        IPAddress ip = Ethernet.localIP();
        Serial.print("IP Address: "); Serial.println(ip);
        server.begin();
        Serial.print("Browse to:  http://"); Serial.println(ip);
    }

    Serial.println("====================================");
}

// -------------------------------------------------------------------
void loop() {
    logRow();    // SD logging at 10 Hz (non-blocking)
    handleWeb(); // Serve status page if a client connects
}
