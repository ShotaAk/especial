#include "parameters.h"

// Especialのボディ情報
// 仕方なくdefineを使用
#define _TIRE_DIAMETER 0.01355
const float pTIRE_DIAMETER = _TIRE_DIAMETER; // meters
const float pTIRE_RADIUS = _TIRE_DIAMETER/2.0; // meters

// 低電圧検知用の電圧値。下回ると低電圧。
const float pLOW_BATTERY_VOLTAGE = 3.7; // volts

// 区画のサイズ
#define _CELL_DISTANCE 0.090
const float pCELL_DISTANCE = _CELL_DISTANCE; // meters
const float pHALF_CELL_DISTANCE = _CELL_DISTANCE/2.0; // meters

// 探索走行用パラメータ
const float pSEARCH_MAX_SPEED = 0.2; // m/s
const float pSEARCH_TIMEOUT = 2.0; // sec
const float pSEARCH_ACCEL = 1.0; // m/ss

// けつ当て用のパラメータ
const float pKETSU_DISTANCE = 0.003; // m/s
const float pKETSU_TIMEOUT = 0.5; // sec


