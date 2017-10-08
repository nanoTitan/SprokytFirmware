#pragma once

#include "constants.h"
#include <stdint.h>

uint8_t Sonar_init();
void Sonar_update();
void Sonar_RegisterDistanceCallback(DistanceCallback callback);