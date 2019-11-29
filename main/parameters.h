#ifndef PARAMETERS_H
#define PARAMETERS_H

// Especialのボディ情報
const float pTIRE_DIAMETER; // meters
const float pTIRE_RADIUS; // meters

// 低電圧検知用の電圧値。下回ると低電圧。
const float pLOW_BATTERY_VOLTAGE; // volts

// 区画のサイズ
const float pCELL_DISTANCE; // meters
const float pHALF_CELL_DISTANCE; // meters

// 探索走行用パラメータ
const float pSEARCH_MAX_SPEED; // m/s
const float pSEARCH_TIMEOUT; // sec
const float pSEARCH_ACCEL; // m/ss

// 最短走行用パラメータ
const float pFAST_MAX_SPEED; // m/s
const float pFAST_TIMEOUT; // sec
const float pFAST_ACCEL; // m/ss

// けつ当て用のパラメータ
const float pKETSU_DISTANCE; // m/s
const float pKETSU_TIMEOUT; // sec

#endif
