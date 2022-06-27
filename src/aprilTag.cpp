#include <Arduino.h>
#include "april_tag.hpp"
#include "defines.hpp"
#include "telnet_debug.hpp"

namespace
{
    const uint8_t magic1[4] = {0x41, 0x50, 0x52, 0x49};
    const uint8_t magic2[4] = {0x4c, 0x54, 0x41, 0x47};
    const uint8_t version[4] = {0x00, 0x01, 0x00, 0x02};
    unsigned long timeoutTimer = 0;
};

unsigned int detectTagCenter = 0;
int numTags = 0;
bool udpConnection = false;

void testTimeout()
{
    if (udpConnection && millis() - timeoutTimer > UDP_TIMEOUT)
    {
        DEBUG_MSG("UDP timeout");
        udpConnection = false;
    }
}

/**
 * @brief print the content of an AprilTag packet to the serial monitor
 *
 */
void AprilTag::print()
{
    Serial.printf("AprilTag: id: %d hamming: %d ncodes: %d c: %f/%f\n",
                  id, hamming, ncodes, c[0], c[1]);
}

/**
 * @brief converts a uint8_t array to an int
 *
 * @param buffer a pointer to an array of uint8_t
 * @return int
 */
int buffToInteger(uint8_t *buffer)
{
    int i = 0;
    uint8_t *iPtr = (uint8_t *)&i;
    iPtr[0] = buffer[3];
    iPtr[1] = buffer[2];
    iPtr[2] = buffer[1];
    iPtr[3] = buffer[0];
    return i;
}

/**
 * @brief converts a uint8_t array to a float
 *
 * @param buffer a pointer to an array of uint8_t
 * @return int
 */
float buffToFloat(uint8_t *buffer)
{
    float f;
    uint8_t *fPtr = (uint8_t *)&f;
    fPtr[0] = buffer[3];
    fPtr[1] = buffer[2];
    fPtr[2] = buffer[1];
    fPtr[3] = buffer[0];
    return f;
}

/**
 * @brief test wether a AsyncUDPPacket contains AprilTag data
 *
 * @param packet a AsyncUDPPacket
 * @return true or false
 */
bool testApril(AsyncUDPPacket packet)
{
    if (packet.length() < 24)
    {
        Serial.print("wrong length: ");
        Serial.println(packet.length());
        return false;
    }
    // test Magic Numbers
    for (int i = 0; i < 4; i++)
    {
        if (magic1[i] != packet.data()[i])
        {
            Serial.println("wrong Magic1!");
            return false;
        }
        if (magic2[i] != packet.data()[4 + i])
        {
            Serial.println("wrong Magic2!");
            return false;
        }
        if (version[i] != packet.data()[8 + i])
        {
            Serial.println("wrong Version!");
            return false;
        }
    }
    return true;
}

/**
 * @brief parse a AsyncUDPPacket packet containing AprilTag data
 *
 * @param packet a AsyncUDPPacket
 */
void parseApril(AsyncUDPPacket packet)
{
    if (!udpConnection)
    {
        DEBUG_MSG("udp connected");
        udpConnection = true;
    }
    timeoutTimer = millis();
    // Serial.println("April Packet: ");
    for (int i = 12; i < packet.length(); i++)
    {
        // Serial.print(packet.data()[i], HEX);
        // Serial.print(":");
    }
    // Serial.println();
    numTags = buffToInteger(packet.data() + 12);
    detectTagCenter = 0;
    double tagCenterTotal = 0;
    if (numTags > 0)
    {
        uint8_t *tagPTemp = packet.data() + 24;
        for (int i = 0; i < numTags; i++)
        {
            AprilTag aTag;
            aTag.id = buffToInteger(tagPTemp);
            tagPTemp += 4;
            aTag.hamming = buffToInteger(tagPTemp);
            tagPTemp += 4;
            aTag.ncodes = buffToInteger(tagPTemp);
            tagPTemp += 4;
            for (int i = 0; i < 2; i++)
            {
                aTag.c[i] = buffToFloat(tagPTemp);
                tagPTemp += 4;
            }
            for (int i = 0; i < 4; i++)
            {
                aTag.p[i][0] = buffToFloat(tagPTemp);
                tagPTemp += 4;
                aTag.p[i][1] = buffToFloat(tagPTemp);
                tagPTemp += 4;
            }
            for (int i = 0; i < 9; i++)
            {
                aTag.H[i] = buffToFloat(tagPTemp);
                tagPTemp += 4;
            }
            tagCenterTotal += aTag.c[1];
            //]aTag.print();
        }
        detectTagCenter = tagCenterTotal / numTags;
    }
}