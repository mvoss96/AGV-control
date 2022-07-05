#pragma once

enum robotStatus
{
    ROBOT_IDLE,
    ROBOT_APPROACHING_STATION,
    ROBOT_STOPPED_NEAR_STATION,
    ROBOT_DRIVING_AWAY
};

enum stationStatus
{
    STATION_IDLE,
    STATION_WORKING,
};

enum cargo
{
    CARGO_EMPTY,
    CARGO_GUMMY,
    CARGO_COTTON,
    CARGO_BALL
};

enum request
{
    REQUEST_NO_REQUEST,
    REQUEST_UNLOADING,
    REQUEST_LOAD_GUMMY,
    REQUEST_LOAD_COTTON,
    REQUEST_LOAD_BALL
};

const uint8_t PREAMBLE[3] = {0x41, 0x47, 0x56};

struct agvMsg
{
    uint8_t preamble[3];
    uint8_t robotStatus;
    uint8_t cargo;
    uint8_t request;
};

struct stationMsg
{
    uint8_t preamble[3];
    uint8_t stationStatus;
};

void wifiSetup();

unsigned getNumClients();

void sendAgvPck(uint8_t status, uint8_t cargo, uint8_t request);

