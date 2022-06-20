#include "WiFi.h"
#include "AsyncUDP.h"
#include "wifi.hpp"
#include "defines.hpp"
#include "april_tag.hpp"
//#include "ESPAsyncWebServer.h"

AsyncUDP udp;

void udpOnPck(AsyncUDPPacket packet)
{
    if (testApril(packet))
    {
        parseApril(packet);
    }

}

void wifiSetup()
{
    Serial.print("Wifi-Start-AP... ");
    WiFi.mode(WIFI_AP);
    WiFi.disconnect();
    delay(100);
    WiFi.softAP(WIFI_SSID, "");
    Serial.print("IP address: ");
    Serial.println(WiFi.softAPIP());
    if (udp.listen(UDP_PORT))
    {
        udp.onPacket(udpOnPck);
        Serial.print("Start listening for udp packets on port: ");
        Serial.println(UDP_PORT);
    }
}
