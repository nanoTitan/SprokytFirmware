#pragma once

#include "constants.h"

enum SquareTestStep
{
	ST_Side1 = 0,
	ST_Turn1,
	ST_Side2,
	ST_Turn2,
	ST_Side3,
	ST_Turn3,
	ST_Side4,
	ST_Turn4
};

enum SquareTestStepDir
{
	ST_CW  = 0,
	ST_CCW,
};

void SquareTest_start(enum SquareTestStepDir dir, float length, uint32_t stepPauseTime);
void SquareTest_update();
