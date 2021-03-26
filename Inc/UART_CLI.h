/*
 FreeRTOS V7.3.0 - Copyright (C) 2012 Real Time Engineers Ltd.


 ***************************************************************************
 *                                                                       *
 *    FreeRTOS tutorial books are available in pdf and paperback.        *
 *    Complete, revised, and edited pdf reference manuals are also       *
 *    available.                                                         *
 *                                                                       *
 *    Purchasing FreeRTOS documentation will not only help you, by       *
 *    ensuring you get running as quickly as possible and with an        *
 *    in-depth knowledge of how to use FreeRTOS, it will also help       *
 *    the FreeRTOS project to continue with its mission of providing     *
 *    professional grade, cross platform, de facto standard solutions    *
 *    for microcontrollers - completely free of charge!                  *
 *                                                                       *
 *    >>> See http://www.FreeRTOS.org/Documentation for details. <<<     *
 *                                                                       *
 *    Thank you for using FreeRTOS, and thank you for your support!      *
 *                                                                       *
 ***************************************************************************


 This file is part of the FreeRTOS distribution.

 FreeRTOS is free software; you can redistribute it and/or modify it under
 the terms of the GNU General Public License (version 2) as published by the
 Free Software Foundation AND MODIFIED BY the FreeRTOS exception.
 >>>NOTE<<< The modification to the GPL is included to allow you to
 distribute a combined work that includes FreeRTOS without being obliged to
 provide the source code for proprietary components outside of the FreeRTOS
 kernel.  FreeRTOS is distributed in the hope that it will be useful, but
 WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY
 or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 more details. You should have received a copy of the GNU General Public
 License and the FreeRTOS license exception along with FreeRTOS; if not it
 can be viewed here: http://www.freertos.org/a00114.html and also obtained
 by writing to Richard Barry, contact details for whom are available on the
 FreeRTOS WEB site.

 1 tab == 4 spaces!

 http://www.FreeRTOS.org - Documentation, latest information, license and
 contact details.

 http://www.SafeRTOS.com - A version that is certified for use in safety
 critical systems.

 http://www.OpenRTOS.com - Commercial support, development, porting,
 licensing and training services.
 */

#ifndef UART_COMMAND_CONSOLE_H
#define UART_COMMAND_CONSOLE_H

void vUARTCommandConsoleStart(void);

/*-----------------------------------------------------------*/

/*** Here are the commands registered which can be used in the serial console ***/

static portBASE_TYPE prvTimeOutput(char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString);
static portBASE_TYPE prvADCOutput(char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString);
static portBASE_TYPE prvMode(char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString);
static portBASE_TYPE prvSetClock(char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString);
static portBASE_TYPE prvStartStromPiConsole(char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString);
static portBASE_TYPE prvStartStromPiConsoleQuick(char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString);
static portBASE_TYPE prvQuitStromPiConsole(char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString);
static portBASE_TYPE prvSetDate(char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString);
static portBASE_TYPE prvSetConfig(char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString);
static portBASE_TYPE prvShowStatus(char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString);
static portBASE_TYPE prvShowAlarm(char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString);
static portBASE_TYPE prvPowerOff(char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString);
static portBASE_TYPE prvTimeRPi(char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString);
static portBASE_TYPE prvDateRPi(char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString);
static portBASE_TYPE prvStatusRPi(char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString);

/*** Here you can find  how FreeRTOS needs the command registered
 *
 * For example of xSetClock:
 *
 * 	( const int8_t * const ) "set-clock",          -> This is the command which can be typed by the user
 *  ( const int8_t * const ) "set-clock <hour> <minutes> <seconds>:\r\n Set the Clock of the StromPi RTC \r\n\r\n",  -> This is the Help-text which is displayed in the help section of the serial console
 * 	prvSetClock, -> This is the function which is linked in the UART_CLI.c to the command
 *  3 -> This is the amount of paramters the user have to type with the command
 *
 * ***/

static const CLI_Command_Definition_t xSetClock =
{ (const char *) "set-clock", (const char*) "set-clock <hour> <minutes> <seconds>:\r\n Set the Clock of the StromPi RTC \r\n\r\n", prvSetClock, 3 };

static const CLI_Command_Definition_t xSetDate =
{ (const char *) "set-date", (const char *) "set-date <date> <month> <year> <weekday>:\r\n Set the Date of the StromPi RTC-Clock \r\n\r\n", prvSetDate, 4 };

static const CLI_Command_Definition_t xSetConfig =
{ (const char *) "set-config", (const char *) "", prvSetConfig, 2 };

static const CLI_Command_Definition_t xStartStromPiConsole =
{ (const char *) "startstrompiconsole", (const char *) "", prvStartStromPiConsole, 0
//startstrompiconsole:\r\n Terminal-Console wird auf dem StromPi gestartet\r\n\r\n
		};

static const CLI_Command_Definition_t xStartStromPiConsoleQuick =
{ (const char *) "sspc", (const char *) "", prvStartStromPiConsoleQuick, 0
//startstrompiconsole:\r\n Terminal-Console wird auf dem StromPi gestartet\r\n\r\n
		};

static const CLI_Command_Definition_t xQuitStromPiConsole =
{ (const char *) "quit", (const char *) "quit:\r\n Closes the StromPi-Console\r\n\r\n", prvQuitStromPiConsole, 0
//startstrompiconsole:\r\n Terminal-Console wird auf dem StromPi gestartet\r\n\r\n
		};

static const CLI_Command_Definition_t xMode =
{ (const char *) "strompi-mode",
		(const char *) "strompi-mode <mode-number>:\r\n Configures the mode of the StromPi 3:\r\n  Mode 1: mUSB -> Wide\r\n  Mode 2: Wide -> mUSB\r\n  Mode 3: mUSB -> Battery\r\n  Mode 4: Wide -> Battery\r\n  Mode 5: mUSB -> Wide -> Battery\r\n  Mode 6: Wide -> mUSB -> Battery\r\n\r\n", prvMode, 1 };

static const CLI_Command_Definition_t xADCOutput =
{ (const char *) "adc-output", (const char *) "adc-output:\r\n Outputs the measured Voltages\r\n\r\n", prvADCOutput, 0 };

static const CLI_Command_Definition_t xTimeOutput =
{ (const char *) "time-output", (const char *) "time-output:\r\n Displays the actual time of the StromPi RTC-Clock\r\n\r\n", prvTimeOutput, 0 };

static const CLI_Command_Definition_t xShowStatus =
{ (const char *) "show-status", (const char *) "show-status:\r\n Outputs the actual Global-Configuration\r\n\r\n", prvShowStatus, 0 };

static const CLI_Command_Definition_t xShowAlarm =
{ (const char *) "show-alarm", (const char *) "show-alarm:\r\n Outputs the actual Alarm-Configuration\r\n\r\n", prvShowAlarm, 0 };

static const CLI_Command_Definition_t xPowerOff =
{ (const char *) "poweroff", (const char *) "poweroff:\r\n Shutdown the Raspberry Pi with the StromPi \r\n\r\n", prvPowerOff, 0 };

static const CLI_Command_Definition_t xTimeRPi =
{ (const char *) "time-rpi", (const char *) "", prvTimeRPi, 0 };

static const CLI_Command_Definition_t xDateRPi =
{ (const char *) "date-rpi", (const char *) "", prvDateRPi, 0 };

static const CLI_Command_Definition_t xStatusRPi =
{ (const char *) "status-rpi", (const char *) "", prvStatusRPi, 0 };

int ascii2int(const char* s);

#endif /* UART_COMMAND_CONSOLE_H */

