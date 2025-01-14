/*
* TMC2160.h
*
*  Created on: 13.08.2018
*      Author: LK
*/

#ifndef TMC_IC_TMC2160_H_
#define TMC_IC_TMC2160_H_

#include "tmc/helpers/API_Header.h"
#include "TMC2160_Constants.h"
#include "TMC2160_Fields.h"
#include "TMC2160_Register.h"

// Helper macros
#define TMC2160_FIELD_READ(tdef, address, mask, shift) \
	FIELD_GET(tmc2160_readInt(tdef, address), mask, shift)
#define TMC2160_FIELD_WRITE(tdef, address, mask, shift, value) \
	(tmc2160_writeInt(tdef, address, FIELD_SET(tmc2160_readInt(tdef, address), mask, shift, value)))

typedef struct
{
	ConfigurationTypeDef *config;
	int32_t registerResetState[TMC2160_REGISTER_COUNT];
	uint8_t registerAccess[TMC2160_REGISTER_COUNT];
} TMC2160TypeDef;

typedef void (*tmc2160_callback)(TMC2160TypeDef*, ConfigState);

// Default Register values
#define R10 0x00070A03  // IHOLD_IRUN
#define R6C 0x10410153  // CHOPCONF
#define R70 0xC40C001E  // PWMCONF

// Register access permissions:
// 0x00: none (reserved)
// 0x01: read
// 0x02: write
// 0x03: read/write
// 0x11: read to clear
// 0x42: write, has hardware presets on reset
//static const uint8_t tmc2160_defaultRegisterAccess[TMC2160_REGISTER_COUNT] = {
////  0     1     2     3     4     5     6     7     8     9     A     B     C     D     E     F
//	0x03, 0x23, ____, ____, 0x01, ____, ____, ____, ____, ____, ____, ____, ____, ____, ____, ____, // 0x00 - 0x0F
//	0x02, 0x02, 0x01, 0x02, 0x02, 0x02, ____, ____, ____, ____, ____, ____, ____, ____, ____, ____, // 0x10 - 0x1F
//	____, ____, ____, ____, ____, ____, ____, ____, ____, ____, ____, ____, ____, 0x03, ____, ____, // 0x20 - 0x2F
//	____, ____, ____, 0x02, ____, ____, ____, ____, ____, ____, ____, ____, ____, ____, ____, ____, // 0x30 - 0x3F
//	____, ____, ____, ____, ____, ____, ____, ____, ____, ____, ____, ____, ____, ____, ____, ____, // 0x40 - 0x4F
//	____, ____, ____, ____, ____, ____, ____, ____, ____, ____, ____, ____, ____, ____, ____, ____, // 0x50 - 0x5F
//	0x42, 0x42, 0x42, 0x42, 0x42, 0x42, 0x42, 0x42, 0x42, 0x42, 0x01, 0x01, 0x03, 0x02, 0x02, 0x01, // 0x60 - 0x6F
//	0x02, 0x01, 0x02, 0x01, ____, ____, ____, ____, ____, ____, ____, ____, ____, ____, ____, ____  // 0x70 - 0x7F
//};

// Register access permissions:
// 0x00: none (reserved)
// 0x01: read
// 0x02: write
// 0x03: read/write
// 0x11: read to clear
// 0x42: write, has hardware presets on reset
static const uint8_t tmc2160_defaultRegisterAccess[TMC2160_REGISTER_COUNT] =
{
//	0     1     2     3     4     5     6     7     8     9     A     B     C     D     E     F
	0x03, 0x23, 0x01, 0x02, 0x23, 0x02, 0x02, 0x01, 0x42, 0x42, 0x42, 0x02, 0x01, ____, ____, ____, // 0x00 - 0x0F
	0x02, 0x02, 0x01, 0x02, 0x02, 0x02, ____, ____, ____, ____, ____, ____, ____, ____, ____, ____, // 0x10 - 0x1F
	0x03, 0x03, 0x01, 0x02, 0x02, 0x02, 0x02, 0x02, 0x02, 0x00, 0x02, 0x02, 0x02, 0x03, ____, ____, // 0x20 - 0x2F
	____, ____, ____, 0x02, 0x03, 0x23, 0x01, ____, 0x03, 0x03, 0x02, 0x23, 0x01, 0x02, ____, ____, // 0x30 - 0x3F
	____, ____, ____, ____, ____, ____, ____, ____, ____, ____, ____, ____, ____, ____, ____, ____, // 0x40 - 0x4F
	____, ____, ____, ____, ____, ____, ____, ____, ____, ____, ____, ____, ____, ____, ____, ____, // 0x50 - 0x5F
	0x42, 0x42, 0x42, 0x42, 0x42, 0x42, 0x42, 0x42, 0x42, 0x42, 0x01, 0x01, 0x03, 0x02, 0x02, 0x01, // 0x60 - 0x6F
	0x02, 0x01, 0x01, 0x01, ____, ____, ____, ____, ____, ____, ____, ____, ____, ____, ____, ____  // 0x70 - 0x7F
};

static const int32_t tmc2160_defaultRegisterResetState[TMC2160_REGISTER_COUNT] =
{
//	0,   1,   2,   3,   4,   5,   6,   7,   8,   9,   A,   B,   C,   D,   E,   F
	0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0, // 0x00 - 0x0F
	R10, 0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0, // 0x10 - 0x1F
	0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0, // 0x20 - 0x2F
	0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0, // 0x30 - 0x3F
	0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0, // 0x40 - 0x4F
	0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0, // 0x50 - 0x5F
	N_A, N_A, N_A, N_A, N_A, N_A, N_A, N_A, N_A, N_A, 0,   0,   R6C, 0,   0,   0, // 0x60 - 0x6F
	R70, 0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0, // 0x70 - 0x7F
};

// Undefine the default register values.
// This prevents warnings in case multiple TMC-API chip headers are included at once
#undef R10
#undef R6C
#undef R70

// Register constants (only required for 0x42 registers, since we do not have
// any way to find out the content but want to hold the actual value in the
// shadow register so an application (i.e. the TMCL IDE) can still display
// the values. This only works when the register content is constant.
static const TMCRegisterConstant tmc2160_RegisterConstants[] =
{		// Use ascending addresses!
		{ 0x08, 0x00000000 }, // FACTORY_CONF
		{ 0x09, 0x00000000 }, // SHORT_CONF
		{ 0x0A, 0x00000000 }, // DRV_CONF
		{ 0x60, 0xAAAAB554 }, // MSLUT[0]
		{ 0x61, 0x4A9554AA }, // MSLUT[1]
		{ 0x62, 0x24492929 }, // MSLUT[2]
		{ 0x63, 0x10104222 }, // MSLUT[3]
		{ 0x64, 0xFBFFFFFF }, // MSLUT[4]
		{ 0x65, 0xB5BB777D }, // MSLUT[5]
		{ 0x66, 0x49295556 }, // MSLUT[6]
		{ 0x67, 0x00404222 }, // MSLUT[7]
		{ 0x68, 0xFFFF8056 }, // MSLUTSEL
		{ 0x69, 0x00F70000 }  // MSLUTSTART
};

void tmc2160_writeDatagram(TMC2160TypeDef *tmc2160, uint8_t address, uint8_t x1, uint8_t x2, uint8_t x3, uint8_t x4);
void tmc2160_writeInt(TMC2160TypeDef *tmc2160, uint8_t address, int32_t value);
int32_t tmc2160_readInt(TMC2160TypeDef *tmc2160, uint8_t address);

void tmc2160_init(TMC2160TypeDef *tmc2160, uint8_t channel, ConfigurationTypeDef *config, const int32_t *registerResetState);
void tmc2160_fillShadowRegisters(TMC2160TypeDef *tmc2160);
uint8_t tmc2160_reset(TMC2160TypeDef *tmc2160);
uint8_t tmc2160_restore(TMC2160TypeDef *tmc2160);
void tmc2160_setRegisterResetState(TMC2160TypeDef *tmc2160, const int32_t *resetState);
void tmc2160_setCallback(TMC2160TypeDef *tmc2160, tmc2160_callback callback);
void tmc2160_periodicJob(TMC2160TypeDef *tmc2160, uint32_t tick);

#endif /* TMC_IC_TMC2160_H_ */
