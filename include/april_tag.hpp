#pragma once
#include "AsyncUDP.h"

struct AprilTag
{
    int id;
    int hamming;
    int ncodes;
    double c[2];
    double p[4][2];
    double H[9];
    void print();
};

bool testApril(AsyncUDPPacket packet);
void parseApril(AsyncUDPPacket packet);
void testTimeout();


