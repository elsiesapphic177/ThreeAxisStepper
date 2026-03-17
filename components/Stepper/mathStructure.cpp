#include "mathStructure.h"
#include<esp_log.h>
const char* mathTag = "MathStructure";

std::array<float, CIRCLE_TABLE_RESOLUTION> CircleTable::sinTable;
std::array<float, CIRCLE_TABLE_RESOLUTION> CircleTable::cosTable;
float CircleTable::minDegree;

void CircleTable::init()
{
    minDegree = 360.0f / CIRCLE_TABLE_RESOLUTION;
    for(int i = 0; i < CIRCLE_TABLE_RESOLUTION; i++){
        float angle = (i * minDegree) * degreeToArc;
        sinTable[i] = std::sin(angle);
        cosTable[i] = std::cos(angle);
    }
}

float CircleTable::fastSin(int index)
{
    return sinTable[index % CIRCLE_TABLE_RESOLUTION];
}

float CircleTable::fastCos(int index)
{
    return cosTable[index % CIRCLE_TABLE_RESOLUTION];
}
