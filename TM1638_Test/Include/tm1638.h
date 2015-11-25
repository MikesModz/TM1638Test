/**************************************************************************************
* TM1638.c
*
* TM1638 LED controller with key-scan interface driver for Atmel AVR
* micro controllers.
*
*   Copyright (C) 2015 Michael Williamson. All rights reserved.
*   Authors: Michael Williamson <mikesmodz@gmail.com>
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions
* are met:
*
* 1. Redistributions of source code must retain the above copyright
*    notice, this list of conditions and the following disclaimer.
* 2. Redistributions in binary form must reproduce the above copyright
*    notice, this list of conditions and the following disclaimer in
*    the documentation and/or other materials provided with the
*    distribution.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
* "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
* LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
* FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
* COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
* INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
* BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
* OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
* AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
* LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
* ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
* POSSIBILITY OF SUCH DAMAGE.
*
****************************************************************************/
#ifndef __TM1638_H__
#define __TM1638_H__

/****************************************************************************
* Included Files
****************************************************************************/
#include <avr/io.h>
#include "avrlibtypes.h"

/****************************************************************************
 * Pre-processor Definitions
****************************************************************************/

/* Definitions for clock pin */
#define TM1638_CLOCK_PORT	PORTC
#define TM1638_CLOCK_DIR	DDRC
#define TM1638_CLOCK_PINS	PINC
#define TM1638_CLOCK_PIN	PORTC0
#define TM1638_CLOCK_HIGH	TM1638_CLOCK_PORT |= _BV(TM1638_CLOCK_PIN);
#define TM1638_CLOCK_LOW	TM1638_CLOCK_PORT &= ~_BV(TM1638_CLOCK_PIN);

/* Definitions for strobe pin */
#define TM1638_STROBE_PORT	PORTC
#define TM1638_STROBE_DIR	DDRC
#define TM1638_STROBE_PINS	PINC
#define TM1638_STROBE_PIN	PORTC1
#define TM1638_STROBE_HIGH	TM1638_STROBE_PORT |= _BV(TM1638_STROBE_PIN);
#define TM1638_STROBE_LOW	TM1638_STROBE_PORT &= ~_BV(TM1638_STROBE_PIN);

/* Definitions for data pin */
#define TM1638_DATA_PORT	PORTC
#define TM1638_DATA_DIR		DDRC
#define TM1638_DATA_PINS	PINC
#define TM1638_DATA_PIN		PORTC2
#define TM1638_DATA_HIGH	TM1638_DATA_PORT |= _BV(TM1638_DATA_PIN);
#define TM1638_DATA_LOW		TM1638_DATA_PORT &= ~_BV(TM1638_DATA_PIN);

/* Definitions for TM1638 commands */
#define TM1638_ADDR_INSTRUCTION_SET		0xC0
#define TM1638_DATA_WRITE_AUTO_ADDR		0x40
#define TM1638_DATA_WRITE_FIX_ADDR		0x44
#define TM1638_READ_KEY_SCAN_MODE		0x42
#define TM1638_DISPLAY_CONTROL_SET		0x80	

/****************************************************************************
* Public Function Prototypes
****************************************************************************/
void TM1638_Init(const u08 intensity);
void TM1638_ClearDigits(void);
void TM1638_SetLED(const u08 pos, const u08 colour);
void TM1638_SetDigit(const u08 pos, const u08 segments, u08 dot);
u08 TM1638_GetKeys(void);
void TM1638_SetDisplayDigit(const u08 digit, const u08 pos, u08 dot);
void TM1638_ClearDisplayDigit(const u08 pos, u08 dot);
void TM1638_SendData(const u08 adr, const u08 data);

#endif /* __TM1638_H__ */
