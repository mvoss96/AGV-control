#include "WiFi.h"
#include "AsyncUDP.h"
#include "wifi.hpp"
#include "defines.hpp"
#include "april_tag.hpp"
#include "ESPTelnet.h"
#include "esp_wifi.h"

ESPTelnet telnet;
bool telnetConnection = false;
uint8_t missionMode = missions::NO_MISSION;
unsigned robotStatus = ROBOT_IDLE;
unsigned robotCargo = CARGO_EMPTY;
unsigned robotRequest = REQUEST_NO_REQUEST;

namespace
{
    AsyncUDP udp1;
    AsyncUDP udp2;
    wifi_sta_list_t wifi_sta_list;
    tcpip_adapter_sta_list_t adapter_sta_list;
    bool stationIsWorking = false;

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
        else if (input == "da" && missionMode != missions::NO_MISSION)
        {
            DEBUG_MSG("set mission to DRIVING_AWAY");
            missionMode = missions::DRIVING_AWAY;
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
            vTaskDelay(0);
        }
        Serial.println("udpTimeoutTask closed");
        vTaskDelete(NULL);
    }

    void udpCommTask(void *argument)
    {
        Serial.print("udpCommTask is running on: ");
        Serial.println(xPortGetCoreID());

        for (;;)
        {
            sendAgvPck(robotStatus, robotCargo, robotRequest);
            delay(500);
        }
        Serial.println("udpCommTask closed");
        vTaskDelete(NULL);
    }

    void udpOnPck(AsyncUDPPacket packet)
    {
        if (testApril(packet))
        {
            parseApril(packet);
        }
    }

    void printPacket(AsyncUDPPacket packet)
    {
        for (size_t i = 0; i < packet.length() - 2; i++)
        {
            Serial.printf(" %x", packet.data()[i]);
        }
        Serial.println();
    }

    /**
     * @brief returns wether a UDP packet contains the correct Preamble
     *
     * @param AsyncUDPPacket
     * @return true||false
     */
    bool testPreamble(AsyncUDPPacket packet)
    {
        for (int i = 0; i < sizeof(PREAMBLE); i++)
        {
            if (packet.data()[i] != PREAMBLE[i])
            {
                Serial.print("wrong preamble! ");
                printPacket(packet);
                return false;
            }
        }
        return true;
    }

    /**
     * @brief returns wether a AsyncUDP packet is a valid station->AGV commPck
     *
     * @param AsyncUDPPacket
     * @return true||false
     */
    bool testStationCommPck(AsyncUDPPacket packet)
    {
        // udp packets are alyways two bytes longer then the data
        if (packet.length() - 2 != sizeof(stationMsg))
        {
            Serial.printf("wrong packet length: %i ", packet.length() - 2);
            printPacket(packet);
            return false;
        }
        return testPreamble(packet);
    }

    /**
     * @brief return the number of connected Clients
     *
     * @return number of clients
     */
    unsigned getNumClients()
    {
        // get connected IPs
        esp_wifi_ap_get_sta_list(&wifi_sta_list);
        tcpip_adapter_get_sta_list(&wifi_sta_list, &adapter_sta_list);
        return adapter_sta_list.num;
    }

    /**
     * @brief gets called on every received UDP packet
     *
     * @param packet
     */
    void udpOnCommPck(AsyncUDPPacket packet)
    {
        if (testStationCommPck(packet))
        {
            Serial.print("Station packet received:");
            printPacket(packet);
            if (missionMode == missions::WAITING && robotStatus == ROBOT_STOPPED_NEAR_STATION)
            {
                if (packet.data()[0] == STATION_WORKING && !stationIsWorking)
                {
                    DEBUG_MSG("station signaled start working");
                    stationIsWorking = true;
                }
                else if (packet.data()[0] == STATION_IDLE && stationIsWorking)
                {
                    DEBUG_MSG("station signaled finished working");
                    stationIsWorking = false;
                    missionMode = missions::DRIVING_AWAY;
                }
            }
        }
    }
}

/**
 * @brief send a agvMsg to the station
 *
 * @param status
 * @param cargo
 * @param request
 */
void sendAgvPck(uint8_t status, uint8_t cargo, uint8_t request)
{
    // get connected IPs
    esp_wifi_ap_get_sta_list(&wifi_sta_list);
    tcpip_adapter_get_sta_list(&wifi_sta_list, &adapter_sta_list);

    // send udp message to evry client
    for (int i = 0; i < adapter_sta_list.num; i++)
    {
        tcpip_adapter_sta_info_t station = adapter_sta_list.sta[i];
        String ip = (ip4addr_ntoa((ip4_addr_t *)&(station.ip)));
        uint8_t msg[6] = {PREAMBLE[0], PREAMBLE[1], PREAMBLE[2], status, cargo, request};
        // udp.connect((ip_addr_t*)&(station.ip),UDP_COMM_PORT);
        // udp.write(msg,6);
        udp2.broadcastTo(msg, 6, 7708);
        // Serial.printf("send to %s", ip);
        // for (size_t i = 0; i < 6; i++)
        // {
        //     Serial.printf(" %x", msg[i]);
        // }
        // Serial.println();
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
    if (udp1.listen(UDP_PORT))
    {
        udp1.onPacket(udpOnPck);
        Serial.print("Start listening for udp packets on port: ");
        Serial.println(UDP_PORT);
    }
    if (udp2.listen(UDP_COMM_PORT))
    {
        udp2.onPacket(udpOnCommPck);
        Serial.print("Start listening for udp comm packets on port: ");
        Serial.println(UDP_COMM_PORT);
    }
    setupTelnet();
    xTaskCreatePinnedToCore(udpTimeoutTask, "udpTimeoutTask", 10000, NULL, 1, NULL, 1);
    xTaskCreatePinnedToCore(udpCommTask, "udpCommTask", 10000, NULL, 1, NULL, 1);
}