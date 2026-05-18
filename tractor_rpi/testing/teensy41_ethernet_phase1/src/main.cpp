// teensy41_ethernet_phase1
// Phase 1: Get an IP address via DHCP, print it to Serial,
//          and serve a simple web page showing the IP address.
//
// Library: QNEthernet by ssilverman (install via PlatformIO lib_deps)
// Hardware: Teensy 4.1 with Ethernet add-on board (RJ45)

#include <Arduino.h>
#include <QNEthernet.h>

using namespace qindesign::network;

EthernetServer server(80);

void setup() {
    Serial.begin(115200);
    while (!Serial && millis() < 4000);

    Serial.println("=== Teensy 4.1 Ethernet Phase 1 ===");

    // Start Ethernet — QNEthernet uses DHCP by default
    Serial.println("Starting Ethernet (DHCP)...");
    Ethernet.begin();

    // Wait up to 15 seconds for a DHCP-assigned IP
    Serial.println("Waiting for DHCP lease...");
    if (!Ethernet.waitForLocalIP(15000)) {
        Serial.println("ERROR: No IP assigned within 15 seconds.");
        Serial.println("Check: cable connected? Router reachable?");
        while (true) {}
    }

    IPAddress ip = Ethernet.localIP();
    Serial.print("IP Address : "); Serial.println(ip);
    Serial.print("Subnet Mask: "); Serial.println(Ethernet.subnetMask());
    Serial.print("Gateway    : "); Serial.println(Ethernet.gatewayIP());

    server.begin();
    Serial.println("HTTP server started on port 80.");
    Serial.print("Browse to:  http://");
    Serial.println(ip);
    Serial.println("====================================");
}

void loop() {
    EthernetClient client = server.accept();
    if (!client) return;

    // Drain the HTTP request (we don't need to parse it for this test)
    unsigned long timeout = millis();
    while (client.connected() && millis() - timeout < 1000) {
        if (client.available()) {
            client.read();
            timeout = millis();
        }
    }

    // Build the response page
    IPAddress ip = Ethernet.localIP();
    String ipStr = String(ip[0]) + "." + String(ip[1]) + "." +
                   String(ip[2]) + "." + String(ip[3]);

    String body =
        "<!DOCTYPE html><html><head><title>Teensy 4.1</title>"
        "<style>"
        "  body { font-family: sans-serif; background: #1e1e1e; color: #e0e0e0;"
        "         display: flex; justify-content: center; align-items: center;"
        "         height: 100vh; margin: 0; }"
        "  .card { background: #2d2d2d; border-radius: 12px; padding: 40px 60px;"
        "          box-shadow: 0 4px 20px rgba(0,0,0,0.5); text-align: center; }"
        "  h1 { color: #7ec8e3; margin-bottom: 8px; }"
        "  .ip { font-size: 2em; font-weight: bold; color: #a8d8a8; margin: 20px 0; }"
        "  .label { color: #888; font-size: 0.9em; }"
        "</style></head><body>"
        "<div class='card'>"
        "  <h1>Teensy 4.1 Ethernet Logger</h1>"
        "  <div class='label'>Phase 1 — IP Address Test</div>"
        "  <div class='ip'>" + ipStr + "</div>"
        "  <div class='label'>HTTP server is running &#10003;</div>"
        "</div>"
        "</body></html>";

    client.println("HTTP/1.1 200 OK");
    client.println("Content-Type: text/html");
    client.print("Content-Length: ");
    client.println(body.length());
    client.println("Connection: close");
    client.println();
    client.print(body);
    client.flush();
    client.stop();

    Serial.print("Page served to client — millis=");
    Serial.println(millis());
}
