#include "WiFi.h"
#include "AsyncUDP.h"
#include "wifi.hpp"
#include "defines.hpp"
#include "april_tag.hpp"
#include "ESPTelnet.h"

ESPTelnet telnet;
bool telnetConnection = false;
uint8_t missionMode = missions::NO_MISSION;

namespace
{
    AsyncUDP udp;
    void onInputReceived(String input)
    {
        Serial.printf("telnet -> %s\n", input);
        if (input == "stop")
        {
            DEBUG_MSG("set mission to NO_MISSION");
            missionMode = missions::NO_MISSION;
        }
        else if (input == "d" && missionMode == missions::NO_MISSION)
        {
            DEBUG_MSG("set mission to DELIVER");
            missionMode = missions::DELIVER;
        }
        else if (input == "gg" && missionMode == missions::NO_MISSION)
        {
            DEBUG_MSG("set mission to GET_GUMMY_BEAR");
            missionMode = missions::GET_GUMMY;
        }
        else if (input == "gc" && missionMode == missions::NO_MISSION)
        {
            DEBUG_MSG("set mission to GET_COTTON_WOOL");
            missionMode = missions::GET_COTTON;
        }
        else if (input == "gb" && missionMode == missions::NO_MISSION)
        {
            DEBUG_MSG("set mission to GET_BALL");
            missionMode = missions::GET_BALL;
        }
        else
        {
            DEBUG_MSG("unknown command");
        }
    }

    void onTelnetConnect(String ip)
    {
        Serial.print("- Telnet: ");
        Serial.print(ip);
        Serial.println(" connected");
        telnet.println("\nWelcome " + telnet.getIP());
        telnetConnection = true;
    }

    void onTelnetDisconnect(String ip)
    {
        Serial.print("- Telnet: ");
        Serial.print(ip);
        Serial.println(" disconnected");
        telnetConnection = false;
    }

    void onTelnetReconnect(String ip)
    {
        Serial.print("- Telnet: ");
        Serial.print(ip);
        Serial.println(" reconnected");
    }

    void onTelnetConnectionAttempt(String ip)
    {
        Serial.print("- Telnet: ");
        Serial.print(ip);
        Serial.println(" tried to connected");
    }

    void setupTelnet()
    {
        // passing on functions for various telnet events
        telnet.onConnect(onTelnetConnect);
        telnet.onConnectionAttempt(onTelnetConnectionAttempt);
        telnet.onReconnect(onTelnetReconnect);
        telnet.onDisconnect(onTelnetDisconnect);
        telnet.onInputReceived(onInputReceived);

        Serial.print("- Telnet: ");
        if (telnet.begin(TELNET_PORT))
        {
            Serial.println("running");
        }
        else
        {
            Serial.println("Telnet error.");
        }
    }

    void udpTimeoutTask(void *argument)
    {
        Serial.print("udpTimeoutTask is running on: ");
        Serial.println(xPortGetCoreID());

        for (;;)
        {
            testTimeout();
            telnet.loop();
            vTaskDelay(10);
        }
        Serial.println("udpTimeoutTask closed");
        vTaskDelete(NULL);
    }

    void udpOnPck(AsyncUDPPacket packet)
    {
        if (testApril(packet))
        {
            parseApril(packet);
        }
    }
}

/**
 * @brief starts wifi AP, udp server and telnet server,
 * also starts a background task for udp timeoutDetection and telnet background loop
 *
 */
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
    setupTelnet();
    xTaskCreatePinnedToCore(udpTimeoutTask, "udpTimeoutTask", 10000, NULL, 1, NULL, 1);
}
