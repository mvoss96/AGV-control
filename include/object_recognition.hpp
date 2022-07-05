#pragma once

enum RecognitedObject
{
    RECOGNITION_ERROR,
    RECOGNITION_NONE,
    RECOGNITION_GUMMY,
    RECOGNITION_BALL,
    RECOGNITION_COTTON
};

bool colorSensorInit();

unsigned measureObject();

void calibrateLux();

bool objectLoaded();