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

/* Standard includes. */
#include "string.h"
#include <inttypes.h>

/*** STM32-HAL Includes ***/

#include "stm32f0xx_hal.h"

extern UART_HandleTypeDef huart1;
extern RTC_HandleTypeDef hrtc;

#include "cmsis_os.h"

/* FreeRTOS includes. */
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"

#include "main.h"

uint8_t rx_ready = 0;
uint8_t console_start = 0;
uint8_t command_always_print = 0;

char firmwareVersion[9] = "v1.72c";

/* FreeRTOS+IO includes. */

/* Example includes. */
#include "FreeRTOS_CLI.h"
#include <UART_CLI.h>

/* Useful stuff */
#include <timedate.h>


/* Dimensions the buffer into which input characters are placed. */
#define cmdMAX_INPUT_SIZE			50

/* Place holder for calls to ioctl that don't use the value parameter. */
#define cmdPARAMTER_NOT_USED		( ( void * ) 0 )

/* Block times of 50 and 500milliseconds, specified in ticks. */
#define cmd50ms						( ( void * ) ( 50UL / portTICK_RATE_MS ) )
#define cmd500ms					( ( void * ) ( 500UL / portTICK_RATE_MS ) )

const char *dashline = "\r\n------------------------------\r\n";
const char *starline = "****************************\r\n";

const char * const __attribute__((section(".text#"))) state_en_dis[2] = {
		"Disabled",
		"Enabled"
};

const char * const __attribute__((section(".text#"))) output_status_msg[4] = {
		"Power-Off",
		"mUSB",
		"Wide",
		"Battery"
};

const char * const __attribute__((section(".text#"))) threeStageMode_msg[2] = {
		"mUSB -> Wide -> Battery",
		"Wide -> mUSB -> Battery"
};

/* TODO: Obviously here is a logic flaw (see last two entries) */
const char * const __attribute__((section(".text#"))) dualStageMode_msg[6] = {
		"mUSB -> Wide",
		"Wide -> mUSB",
		"mUSB -> Battery",
		"Wide -> Battery",
		"mUSB -> Wide -> Battery",
		"Wide -> mUSB -> Battery"
};


const char * const __attribute__((section(".text#"))) batlevel_msg[5] = {
		"Disabled",
		"10%",
		"25%",
		"50%",
		"100%"
};
/*-----------------------------------------------------------*/

/*
 * The task that implements the command console processing.
 */
static void prvUARTCommandConsoleTask(void *pvParameters);
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart);

/*-----------------------------------------------------------*/

/* Holds the handle of the task that implements the UART command console. */
static xTaskHandle xCommandConsoleTask = NULL;

static const int8_t * const pcNewLine = (int8_t *) "\r\n";
static const int8_t * const pcEndOfCommandOutputString = (int8_t *) "\r\n>";

/*-----------------------------------------------------------*/
osThreadId UARTCmdTaskHandle;

void vUARTCommandConsoleStart(void)
{

	/*** Creates the FreeRTOS Task for the Serial Console ***/

	xTaskCreate(prvUARTCommandConsoleTask, /* The task that implements the command console. */
	"UARTCmd", /* Text name assigned to the task.  This is just to assist debugging.  The kernel does not use this name itself. */
	configUART_COMMAND_CONSOLE_STACK_SIZE, /* The size of the stack allocated to the task. */
	NULL, /* The parameter is not used, so NULL is passed. */
	configUART_COMMAND_CONSOLE_TASK_PRIORITY,/* The priority allocated to the task. */
	&xCommandConsoleTask); /* Used to store the handle to the created task. */

	/*** The available Commands are registered here.
	 * Please refer to the definition of the commands here at the bottom of this file (UART_CLI.c)
	 * and to the headerfile (UART_CLI.h)  ***/

	FreeRTOS_CLIRegisterCommand(&xTimeOutput);
	FreeRTOS_CLIRegisterCommand(&xADCOutput);
	FreeRTOS_CLIRegisterCommand(&xMode);
	FreeRTOS_CLIRegisterCommand(&xSetClock);
	FreeRTOS_CLIRegisterCommand(&xSetDate);
	FreeRTOS_CLIRegisterCommand(&xSetConfig);
	FreeRTOS_CLIRegisterCommand(&xStartStromPiConsole);
	FreeRTOS_CLIRegisterCommand(&xStartStromPiConsoleQuick);
	FreeRTOS_CLIRegisterCommand(&xShowStatus);
	FreeRTOS_CLIRegisterCommand(&xShowAlarm);
	FreeRTOS_CLIRegisterCommand(&xPowerOff);
	FreeRTOS_CLIRegisterCommand(&xTimeRPi);
	FreeRTOS_CLIRegisterCommand(&xDateRPi);
	FreeRTOS_CLIRegisterCommand(&xStatusRPi);
	FreeRTOS_CLIRegisterCommand(&xQuitStromPiConsole);

}

/*-----------------------------------------------------------*/
/*** This defines the UART Console Task
 * Its main function is to check the serial interface for incoming Characters
 * which then are stored into an Receive-Buffer for processing.
 * If the console is enabled (through the flag "console_start==1"),
 * then the received character is echoed back through the serial interface,
 * so the user can see what has been typed into the console.
 * After the "Enter"-Key has been pressed (Character '\r')
 * the string which is stored in the ReceiveBuffer is sent
 * to the FreeRTOS+CLI subprocess, which compare it to the preregistered
 * command and process the command if it finds a match.
 * The output of the command is then sent out to the serial
 * interface, when the console is enabled - there a special
 * commands (like dateRpi) which can bypass a deactivated console
 * through the flag command_always_print=1. This is used to communicate
 * with the StromPi3 through scripts, where an input into the console
 * isn't needed ***/

static void prvUARTCommandConsoleTask(void *pvParameters)
{
	uint8_t cRxedChar, cInputIndex = 0;
	char *pcOutputString;
	static char cInputString[cmdMAX_INPUT_SIZE], cLastInputString[cmdMAX_INPUT_SIZE];
	portBASE_TYPE xReturned;

	(void) pvParameters;

	/* Obtain the address of the output buffer.  Note there is no mutual
	 exclusion on this buffer as it is assumed only one command console
	 interface will be used at any one time. */
	pcOutputString = FreeRTOS_CLIGetOutputBuffer();

	/* Send the welcome message. */
	//volatile UBaseType_t uxHighWaterMark;

	for (;;)
	{
		/* Only interested in reading one character at a time. */

		//uxHighWaterMark = uxTaskGetStackHighWaterMark( NULL);

		/*** Process the Serial Interface Interrupt and copy a received Character
		 * into the predesignated buffer.
		 * The whole task waits here in the while-loop until an interrupt gives a
		 * signal for a processed character. ***/
		while (rx_ready != 1)
		{
			HAL_UART_Receive_IT(&huart1, &cRxedChar, 1);
		}
		rx_ready = 0;

		/* Echo the character back. */
		if (UART_CheckIdleState(&huart1) == HAL_OK && console_start == 1)
		{
			HAL_UART_Transmit(&huart1, &cRxedChar, sizeof(cRxedChar), sizeof(cRxedChar));
		}

		/*** Return-Key has been pressed ***/
		if (cRxedChar == '\r')
		{
			/* The input command string is complete.  Ensure the previous
			 UART transmission has finished before sending any more data.
			 This task will be held in the Blocked state while the Tx completes,
			 if it has not already done so, so no CPU time will be wasted by
			 polling. */
			if (UART_CheckIdleState(&huart1) == HAL_OK && console_start == 1)
			{
				HAL_UART_Transmit(&huart1, (uint8_t *) pcNewLine, strlen((char *) pcNewLine), strlen((char *) pcNewLine));
			}
			/* See if the command is empty, indicating that the last command is
			 to be executed again. */
			if (cInputIndex == 0)
			{
				strcpy((char *) cInputString, (char *) cLastInputString);
			}

			/* Pass the received command to the command interpreter.  The
			 command interpreter is called repeatedly until it returns
			 pdFALSE as it might generate more than one string. */
			do
			{
				/* Once again, just check to ensure the UART has completed
				 sending whatever it was sending last.  This task will be held
				 in the Blocked state while the Tx completes, if it has not
				 already done so, so no CPU time	is wasted polling. */
				if (UART_CheckIdleState(&huart1) == HAL_OK)
					xReturned = pdPASS;

				if (xReturned == pdPASS)
				{
					/* Get the string to write to the UART from the command
					 interpreter. */
					xReturned = FreeRTOS_CLIProcessCommand(cInputString, pcOutputString, configCOMMAND_INT_MAX_OUTPUT_SIZE);

					/* Write the generated string to the UART. */
					if (console_start == 1 || command_always_print == 1)
					{
						HAL_UART_Transmit(&huart1, (uint8_t *) pcOutputString, strlen((char *) pcOutputString), strlen((char *) pcOutputString));
						command_always_print = 0;
					}
				}

			} while (xReturned != pdFALSE);

			/* All the strings generated by the input command has been sent.
			 Clear the input	string ready to receive the next command.  Remember
			 the command that was just processed first in case it is to be
			 processed again. */
			strcpy((char *) cLastInputString, (char *) cInputString);
			cInputIndex = 0;
			memset(cInputString, 0x00, cmdMAX_INPUT_SIZE);

			/* Ensure the last string to be transmitted has completed. */
			if (UART_CheckIdleState(&huart1) == HAL_OK && console_start == 1)
			{
				HAL_UART_Transmit(&huart1, (uint8_t *) pcEndOfCommandOutputString, strlen((char *) pcEndOfCommandOutputString), strlen((char *) pcEndOfCommandOutputString));

			}
		}
		else
		{
			if (cRxedChar == '\r')
			{
				/* Ignore the character. */
			}
			else if (cRxedChar == '\b')
			{
				/* Backspace was pressed.  Erase the last character in the
				 string - if any. */
				if (cInputIndex > 0)
				{
					cInputIndex--;
					cInputString[cInputIndex] = '\0';
				}
			}
			else
			{
				/* A character was entered.  Add it to the string
				 entered so far.  When a \n is entered the complete
				 string will be passed to the command interpreter. */
				if ((cRxedChar >= ' ') && (cRxedChar <= '~'))
				{
					if (cInputIndex < cmdMAX_INPUT_SIZE)
					{
						cInputString[cInputIndex] = cRxedChar;
						cInputIndex++;
					}
				}
			}
		}
	}
}

/*-----------------------------------------------------------*/

/*** In the following section you'll find the definition of the preregistered Commands
 * Please refer also to the UART_CLI.h file***/

/*** prvADCOutput
 * This command outputs the measured Voltages which are connected to the StromPi3 ***/

static portBASE_TYPE prvADCOutput(char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString)
{
	(void) pcCommandString;
	configASSERT(pcWriteBuffer);

	/* This function assumes the buffer length is adequate. */
	(void) xWriteBufferLen;

	sprintf(pcWriteBuffer, "%sWide-Range-Inputvoltage: ", starline);

	if (rawValue[0] > minWide)
	{
		sprintf(pcWriteBuffer + strlen(pcWriteBuffer), "%d.%03d V",
				measuredValue[0] / 1000, measuredValue[0] % 1000);
	}
	else
	{
		sprintf(pcWriteBuffer + strlen(pcWriteBuffer), "not connected");
	}

	if (rawValue[1] > minBatConnect)
	{
		sprintf(pcWriteBuffer + strlen(pcWriteBuffer), "\r\nLifePo4-Batteryvoltage: %d.%03d V",
				measuredValue[1] / 1000, measuredValue[1] % 1000);

		if (batLevel >= 1) {
			sprintf(pcWriteBuffer + strlen(pcWriteBuffer), " [%s]", batlevel_msg[batLevel]);
		}

		if (charging == 1)
		{
			sprintf(pcWriteBuffer + strlen(pcWriteBuffer), " [charging]");
		}
	}
	else
	{
		sprintf(pcWriteBuffer + strlen(pcWriteBuffer), "\r\nLifePo4-Batteryvoltage: not connected");
	}
	if (rawValue[2] > minUSB)
	{
		sprintf(pcWriteBuffer + strlen(pcWriteBuffer), "\r\nmicroUSB-Inputvoltage: %d.%03d V",
				measuredValue[2] / 1000, measuredValue[2] % 1000);
	}
	else
	{
		sprintf(pcWriteBuffer + strlen(pcWriteBuffer), "\r\nmicroUSB-Inputvoltage: not connected");
	}
	sprintf(pcWriteBuffer + strlen(pcWriteBuffer),
			"\r\nOutput-Voltage: %d.%03d V\r\n%s",
			measuredValue[3] / 1000, measuredValue[3] % 1000, starline);

	/* There is no more data to return after this single string, so return
	 pdFALSE. */
	return pdFALSE;
}
/*-----------------------------------------------------------*/

/*** prvMode
 * This command changes the mode of the StromPi3, which defines the primary and the secondary backup voltage input
 *
 *  1: mUSB (primary) -> Wide (secondary)
 *  2: Wide (primary) -> mUSB (secondary)
 *  3: mUSB (primary) -> Battery (secondary)
 *  4: Wide (primary) -> Battery (secondary)
 *
 * ***/

static portBASE_TYPE prvMode(char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString)
{
	const char *pcMessage = "%sMode has been changed\r\n%s";
	char *pcParam;
	BaseType_t xParamLen;

	pcParam = (char *)FreeRTOS_CLIGetParameter(pcCommandString, 1, &xParamLen);
	modus = ascii2int(pcParam);

	if (modus < 5)
	{
		threeStageMode = 0;
	}
	else if (modus == 5)
	{
		threeStageMode = 1;
		modus = 1;
	}
	else if (modus == 6)
	{
		threeStageMode = 2;
		modus = 2;
	}

	(void) pcCommandString;
	configASSERT(pcWriteBuffer);

	/* This function assumes the buffer length is adequate. */
	(void) xWriteBufferLen;

	sprintf(pcWriteBuffer, pcMessage, starline, starline);

	/*** The updated "modus"-variable is written into the flash ***/
	flashConfig();

	/* There is no more data to return after this single string, so return
	 pdFALSE. */
	return pdFALSE;
}
/*-----------------------------------------------------------*/

/*** prvTimeOutput
 * This command shows the actual time of the STM32 RTC-Module
 * ***/

__attribute__((unused))
static portBASE_TYPE prvTimeOutput(char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString)
{

	(void) pcCommandString;
	configASSERT(pcWriteBuffer);

	RTC_TimeTypeDef stimestructureget;
	RTC_DateTypeDef sdatestructureget;

	HAL_RTC_GetTime(&hrtc, &stimestructureget, RTC_FORMAT_BIN);
	HAL_RTC_GetDate(&hrtc, &sdatestructureget, RTC_FORMAT_BIN);

	/* This function assumes the buffer length is adequate. */
	(void) xWriteBufferLen;

	sprintf(pcWriteBuffer, "%02d:%02d:%02d",
			stimestructureget.Hours, stimestructureget.Minutes, stimestructureget.Seconds);

	/* There is no more data to return after this single string, so return
	 pdFALSE. */
	return pdFALSE;
}
/*-----------------------------------------------------------*/

/*** prvSetClock
 * This command changes the time which is currently used in the STM32 RTC-Module
 *
 * ***/

__attribute__((unused))
static portBASE_TYPE prvSetClock(char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString)
{

	uint8_t hour;
	uint8_t min;
	uint8_t sec;

	char *pcParams[3];
	BaseType_t xParamLens[3];

	for (int i = 0; i < 3; i++) {
		pcParams[i] = (char *)FreeRTOS_CLIGetParameter(pcCommandString, i, &xParamLens[i]);
		pcParams[i][xParamLens[i]] = 0;
	}

	(void) pcCommandString;
	configASSERT(pcWriteBuffer);

	/* This function assumes the buffer length is adequate. */
	(void) xWriteBufferLen;


	hour = ascii2int(pcParams[0]);
	min = ascii2int(pcParams[1]);
	sec = ascii2int(pcParams[2]);

	RTC_TimeTypeDef stimestructure;

	stimestructure.Hours = hour;
	stimestructure.Minutes = min;
	stimestructure.Seconds = sec;
	stimestructure.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
	stimestructure.StoreOperation = RTC_STOREOPERATION_RESET;

	if (HAL_RTC_SetTime(&hrtc, &stimestructure, RTC_FORMAT_BIN) != HAL_OK)
	{
		/* Initialization Error */
		Error_Handler();
	}

	sprintf(pcWriteBuffer, "The clock has been set to %02d:%02d:%02d",
			stimestructure.Hours, stimestructure.Minutes, stimestructure.Seconds);

	/* There is no more data to return after this single string, so return
	 pdFALSE. */
	return pdFALSE;
}

/*-----------------------------------------------------------*/

/*** prvSetDate
 * This command changes the date which is currently used in the STM32 RTC-Module
 *
 * ***/

static portBASE_TYPE prvSetDate(char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString)
{

	uint8_t day;
	uint8_t month;
	uint8_t year;
	uint8_t weekday;

	char *pcParams[4];
	BaseType_t xParamLens[4];

	for (int i = 0; i < 4; i++) {
		pcParams[i] = (char *)FreeRTOS_CLIGetParameter(pcCommandString, i, &xParamLens[i]);
		pcParams[i][xParamLens[i]] = 0;
	}

	(void) pcCommandString;
	configASSERT(pcWriteBuffer);

	/* This function assumes the buffer length is adequate. */
	(void) xWriteBufferLen;

	day = ascii2int(pcParams[0]);
	month = ascii2int(pcParams[1]);
	year = ascii2int(pcParams[2]);
	weekday = ascii2int(pcParams[3]);

	RTC_DateTypeDef sdatestructure;

	sdatestructure.Year = year;
	sdatestructure.Month = month;
	sdatestructure.Date = day;
	sdatestructure.WeekDay = weekday;

	if (HAL_RTC_SetDate(&hrtc, &sdatestructure, RTC_FORMAT_BIN) != HAL_OK)
	{
		/* Initialization Error */
		Error_Handler();
	}

	sprintf(pcWriteBuffer, "The date has been set to %s %02d.%02d.20%02d", getweekday(weekday),
			sdatestructure.Date, sdatestructure.Month, sdatestructure.Year);

	/* There is no more data to return after this single string, so return
	 pdFALSE. */
	return pdFALSE;
}

/*-----------------------------------------------------------*/

/*** prvSetConfig
 * This command sets the seconds for the Shutdown-Timer
 *
 * ***/

__attribute__((unused))
static portBASE_TYPE prvSetConfig(char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString)
{
	const char * pcMessage = "";

	char *pcParameter1;
	BaseType_t xParameter1StringLength;

	pcParameter1 = (char *)FreeRTOS_CLIGetParameter(pcCommandString, 1, &xParameter1StringLength);

	char *pcParameter2;
	BaseType_t xParameter2StringLength;

	pcParameter2 = (char *)FreeRTOS_CLIGetParameter(pcCommandString, 2, &xParameter2StringLength);

	pcParameter1[xParameter1StringLength] = 0x00;
	pcParameter2[xParameter2StringLength] = 0x00;

	(void) pcCommandString;
	configASSERT(pcWriteBuffer);

	/* This function assumes the buffer length is adequate. */
	(void) xWriteBufferLen;

	uint8_t commandParameter1;
	uint32_t commandParameter2;

	commandParameter1 = ascii2int(pcParameter1);
	commandParameter2 = ascii2int(pcParameter2);

	if (commandParameter1 == 0 && commandParameter2 == 0)
	{
		updateConfig();
	}
	else if (commandParameter1 == 0 && commandParameter2 == 1)
	{
		updateConfig();
		reconfigureWatchdog();
	}
	else if (commandParameter1 == 0 && commandParameter2 == 2)
	{
		if (serialLessMode == 1)
		{
			serialLess_communication_off_counter = 5;
		}
	}
	else if (commandParameter1 == 1)
	{
		configParamters[commandParameter1] = commandParameter2;
		if (commandParameter2 < 5)
		{
			threeStageMode = 0;
		}
		else if (commandParameter2 == 5)
		{
			threeStageMode = 1;
			modus = 1;
		}
		else if (commandParameter2 == 6)
		{
			threeStageMode = 2;
			modus = 2;
		}

	}
	else if (commandParameter1 == 24)
	{
		configParamters[commandParameter1] = commandParameter2;
		if (commandParameter2 == 1)
		{
			if (output_status == 0x2)
			{
				HAL_GPIO_WritePin(CTRL_L7987_GPIO_Port, CTRL_L7987_Pin, GPIO_PIN_SET);
			}
			else
			{
				HAL_GPIO_WritePin(CTRL_L7987_GPIO_Port, CTRL_L7987_Pin, GPIO_PIN_RESET);
			}
		}
	}
	else if (commandParameter1 > 1 && commandParameter1 <= configMax)
	{
		configParamters[commandParameter1] = commandParameter2;
	}

	strcpy(pcWriteBuffer, pcMessage);

	/* There is no more data to return after this single string, so return
	 pdFALSE. */
	return pdFALSE;
}

/*-----------------------------------------------------------*/

static portBASE_TYPE sspc(char *pcWriteBuffer)
{
	const char *pcMessage = "%s Welcome to the StromPi 3 Console %s Type " "help" " to view a list of available commands.\r\n\r\n[When you press ENTER the previous command would be executed again]\r\n";
	configASSERT(pcWriteBuffer);

	/* This function assumes the buffer length is adequate. */
	console_start = 1;

	sprintf(pcWriteBuffer, pcMessage, dashline, dashline);

	return pdFALSE;
}

/*** prvStartStromPiConsole
 *
 * This command enables the console output for the user.
 * The StromPi3 have two possibilities to send out the command-output to the Raspberry Pi
 *
 * - When the console_start flag is set to 1: This means that the user wants to configure the StromPi3 through the serial interface
 * - When the command_always_print flag is set to 1: This means that the output of the command can bypass a deactivated output - this is used to communicate with the StromPi3 through Scripts
 *
 * prvStartStromPiConsole sets the "console_start" flag to 1
 *
 * ***/

__attribute__((unused))
static portBASE_TYPE prvStartStromPiConsole(char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString)
{
	(void) pcCommandString;
	(void) xWriteBufferLen;
	return sspc(pcWriteBuffer);
}

/*-----------------------------------------------------------*/

/*** prvStartStromPiConsoleQuick
 *
 * Same Command as above, but with a shorter Hotword for the activation of the console
 *
 * ***/

__attribute__((unused))
static portBASE_TYPE prvStartStromPiConsoleQuick(char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString)
{
	(void) pcCommandString;
	(void) xWriteBufferLen;
	return sspc(pcWriteBuffer);
}

/*-----------------------------------------------------------*/

/*** prvQuitStromPiConsole
 *
 * This command disables the console output for the user.
 *
 * prvQuitStromPiConsole sets the "console_start" flag to 0
 *
 * ***/

static portBASE_TYPE prvQuitStromPiConsole(char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString)
{
	const char* pcMessage = "";

	(void) pcCommandString;
	configASSERT(pcWriteBuffer);

	/* This function assumes the buffer length is adequate. */
	(void) xWriteBufferLen;

	console_start = 0;

	strcpy((char *) pcWriteBuffer, (char *) pcMessage);

	return pdFALSE;
}

/*-----------------------------------------------------------*/

/*** prvPowerOff
 *
 * With this command it is possible to manual shut down the Raspberry Pi (through the "shutdown"-message)
 * and change the StromPi3 into its PowerOff-state - this would be needed for the Scheduler-System (main.c -> Alarmhandler())
 *
 * ***/

/* defined in main.c */
extern void Config_Reset_Pin_Input_PullDOWN(void);

static portBASE_TYPE prvPowerOff(char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString)
{
	const char* pcMessage = "\r\n Raspberry Pi Shutdown\r\n";

	(void) pcCommandString;
	configASSERT(pcWriteBuffer);

	/* This function assumes the buffer length is adequate. */
	(void) xWriteBufferLen;

	poweroff_flag = 1;
	manual_poweroff_flag = 1;
	alarm_shutdown_enable = 1;
	console_start = 0;

	Config_Reset_Pin_Input_PullDOWN();

	strcpy((char *) pcWriteBuffer, (char *) pcMessage);

	return pdFALSE;
}

/*-----------------------------------------------------------*/

/*** prvTimeRPi
 *
 * This command is for programming the RTC Time through a script. (Like the RTCSerial.py script)
 *
 * It uses the command_always_print=1 flag to bypass a deactivated console_output to communicate directly to the Script executed by the Raspberry Pi
 *
 * ***/

static portBASE_TYPE prvTimeRPi(char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString)
{
	(void) pcCommandString;
	configASSERT(pcWriteBuffer);

	/* This function assumes the buffer length is adequate. */
	(void) xWriteBufferLen;

	uint32_t time;

	RTC_TimeTypeDef stimestructureget;
	RTC_DateTypeDef sdatestructureget;

	HAL_RTC_GetTime(&hrtc, &stimestructureget, RTC_FORMAT_BIN);
	HAL_RTC_GetDate(&hrtc, &sdatestructureget, RTC_FORMAT_BIN);

	time = stimestructureget.Hours * 10000 + stimestructureget.Minutes * 100 + stimestructureget.Seconds;

	command_always_print = 1;

	sprintf((char *) pcWriteBuffer, "%lu", time);

	return pdFALSE;
}

/*-----------------------------------------------------------*/

/*** prvDateRPi
 *
 * This command is for programming the RTC Date through a script. (Like the RTCSerial.py script)
 *
 * It uses the command_always_print=1 flag to bypass a deactivated console_output to communicate directly to the Script executed by the Raspberry Pi
 *
 * ***/

static portBASE_TYPE prvDateRPi(char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString)
{
	(void) pcCommandString;
	configASSERT(pcWriteBuffer);

	/* This function assumes the buffer length is adequate. */
	(void) xWriteBufferLen;

	uint32_t date;

	RTC_TimeTypeDef stimestructureget;
	RTC_DateTypeDef sdatestructureget;

	HAL_RTC_GetTime(&hrtc, &stimestructureget, RTC_FORMAT_BIN);
	HAL_RTC_GetDate(&hrtc, &sdatestructureget, RTC_FORMAT_BIN);

	date = sdatestructureget.Year * 10000 + sdatestructureget.Month * 100 + sdatestructureget.Date;

	command_always_print = 1;

	sprintf((char *) pcWriteBuffer, "%lu", date);

	return pdFALSE;
}

/*-----------------------------------------------------------*/

/*** prvStatusRPi
 *
 * This command is needed for the StromPi3_Status.py script.
 * It is gathering all of the variables currently active and prepares them for transfer
 * through the serial interface.
 *
 * ***/

static portBASE_TYPE prvStatusRPi(char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString)
{
	(void) pcCommandString;
	configASSERT(pcWriteBuffer);

	/* This function assumes the buffer length is adequate. */
	(void) xWriteBufferLen;

	uint32_t time;
	uint32_t date;
	uint8_t alarm_mode_tmp;

	RTC_TimeTypeDef stimestructureget;
	RTC_DateTypeDef sdatestructureget;

	HAL_RTC_GetTime(&hrtc, &stimestructureget, RTC_FORMAT_BIN);
	HAL_RTC_GetDate(&hrtc, &sdatestructureget, RTC_FORMAT_BIN);

	date = sdatestructureget.Year * 10000 + sdatestructureget.Month * 100 + sdatestructureget.Date;
	time = stimestructureget.Hours * 10000 + stimestructureget.Minutes * 100 + stimestructureget.Seconds;

	command_always_print = 1;

	sprintf((char *) pcWriteBuffer, "%lu\n", time);

	sprintf((char *) pcWriteBuffer + strlen((char *) pcWriteBuffer), "%lu\n", date);

	sprintf((char *) pcWriteBuffer + strlen((char *) pcWriteBuffer), "%u\n", sdatestructureget.WeekDay);

	if (threeStageMode > 0)
	{
		sprintf((char *) pcWriteBuffer + strlen((char *) pcWriteBuffer), "%u\n", threeStageMode + 4);
	}
	else
	{
		sprintf((char *) pcWriteBuffer + strlen((char *) pcWriteBuffer), "%u\n", modus);
	}

	sprintf((char *) pcWriteBuffer + strlen((char *) pcWriteBuffer), "%u\n", alarm_enable);

	alarm_mode_tmp = 0;

	if (alarmTime == 1)
		alarm_mode_tmp = 1;
	else if (alarmDate == 1)
		alarm_mode_tmp = 2;
	else if (alarmWeekDay == 1)
		alarm_mode_tmp = 3;

	sprintf((char *) pcWriteBuffer + strlen((char *) pcWriteBuffer), "%u\n", alarm_mode_tmp);

	sprintf((char *) pcWriteBuffer + strlen((char *) pcWriteBuffer), "%u\n", alarm_hour);

	sprintf((char *) pcWriteBuffer + strlen((char *) pcWriteBuffer), "%u\n", alarm_min);

	sprintf((char *) pcWriteBuffer + strlen((char *) pcWriteBuffer), "%u\n", alarm_day);

	sprintf((char *) pcWriteBuffer + strlen((char *) pcWriteBuffer), "%u\n", alarm_month);

	sprintf((char *) pcWriteBuffer + strlen((char *) pcWriteBuffer), "%u\n", alarm_weekday);

	sprintf((char *) pcWriteBuffer + strlen((char *) pcWriteBuffer), "%u\n", alarmPoweroff);

	sprintf((char *) pcWriteBuffer + strlen((char *) pcWriteBuffer), "%u\n", alarm_hour_off);

	sprintf((char *) pcWriteBuffer + strlen((char *) pcWriteBuffer), "%u\n", alarm_min_off);

	sprintf((char *) pcWriteBuffer + strlen((char *) pcWriteBuffer), "%u\n", shutdown_enable);

	sprintf((char *) pcWriteBuffer + strlen((char *) pcWriteBuffer), "%u\n", shutdown_time);

	sprintf((char *) pcWriteBuffer + strlen((char *) pcWriteBuffer), "%u\n", warning_enable);

	sprintf((char *) pcWriteBuffer + strlen((char *) pcWriteBuffer), "%u\n", serialLessMode);

	sprintf((char *) pcWriteBuffer + strlen((char *) pcWriteBuffer), "%u\n", alarmInterval);

	sprintf((char *) pcWriteBuffer + strlen((char *) pcWriteBuffer), "%u\n", alarmIntervalMinOn);

	sprintf((char *) pcWriteBuffer + strlen((char *) pcWriteBuffer), "%u\n", alarmIntervalMinOff);

	sprintf((char *) pcWriteBuffer + strlen((char *) pcWriteBuffer), "%u\n", batLevel_shutdown);

	sprintf((char *) pcWriteBuffer + strlen((char *) pcWriteBuffer), "%u\n", batLevel);

	sprintf((char *) pcWriteBuffer + strlen((char *) pcWriteBuffer), "%u\n", charging);

	sprintf((char *) pcWriteBuffer + strlen((char *) pcWriteBuffer), "%u\n", powerOnButton_enable);

	sprintf((char *) pcWriteBuffer + strlen((char *) pcWriteBuffer), "%u\n", powerOnButton_time);

	sprintf((char *) pcWriteBuffer + strlen((char *) pcWriteBuffer), "%u\n", powersave_enable);

	sprintf((char *) pcWriteBuffer + strlen((char *) pcWriteBuffer), "%u\n", poweroff_enable);

	sprintf((char *) pcWriteBuffer + strlen((char *) pcWriteBuffer), "%u\n", wakeup_time_enable);

	sprintf((char *) pcWriteBuffer + strlen((char *) pcWriteBuffer), "%u\n", wakeup_time);

	sprintf((char *) pcWriteBuffer + strlen((char *) pcWriteBuffer), "%u\n", wakeupweekend_enable);

	sprintf((char *) pcWriteBuffer + strlen((char *) pcWriteBuffer), "%u\n", measuredValue[0]);

	sprintf((char *) pcWriteBuffer + strlen((char *) pcWriteBuffer), "%u\n", measuredValue[1]);

	sprintf((char *) pcWriteBuffer + strlen((char *) pcWriteBuffer), "%u\n", measuredValue[2]);

	sprintf((char *) pcWriteBuffer + strlen((char *) pcWriteBuffer), "%u\n", measuredValue[3]);

	sprintf((char *) pcWriteBuffer + strlen((char *) pcWriteBuffer), "%u\n", output_status);

	sprintf((char *) pcWriteBuffer + strlen((char *) pcWriteBuffer), "%u\n", powerfailure_counter);

	sprintf((char *) pcWriteBuffer + strlen((char *) pcWriteBuffer), firmwareVersion);

	sprintf((char *) pcWriteBuffer + strlen((char *) pcWriteBuffer), "\n");

	return pdFALSE;

}

/*-----------------------------------------------------------*/

/*** prvShowStatus
 * This command sums up all of the configuration data in the StromPi 3
 * and outputs it into the serial console.
 *
 * ***/

static portBASE_TYPE prvShowStatus(char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString)
{
	(void) pcCommandString;
	configASSERT(pcWriteBuffer);

	/* This function assumes the buffer length is adequate. */
	(void) xWriteBufferLen;

	RTC_TimeTypeDef stimestructureget;
	RTC_DateTypeDef sdatestructureget;

	HAL_RTC_GetTime(&hrtc, &stimestructureget, RTC_FORMAT_BIN);
	HAL_RTC_GetDate(&hrtc, &sdatestructureget, RTC_FORMAT_BIN);

	sprintf((char *) pcWriteBuffer, "\r\n Time: %02d:%02d:%02d", stimestructureget.Hours, stimestructureget.Minutes, stimestructureget.Seconds);

	char *temp_message;

	sprintf((char *) pcWriteBuffer + strlen((char *) pcWriteBuffer), "\r\n Date: %s %02d.%02d.20%02d\r\n", getweekday(sdatestructureget.WeekDay),
			sdatestructureget.Date, sdatestructureget.Month, sdatestructureget.Year);



	sprintf((char *) pcWriteBuffer + strlen((char *) pcWriteBuffer), "\r\n StromPi-Output:  %s \r\n",
			output_status_msg[output_status]);



	if (threeStageMode > 0)
	{
		temp_message = (char *)threeStageMode_msg[threeStageMode];
	}
	else
	{
		temp_message = (char *)dualStageMode_msg[modus];
	}

	sprintf((char *) pcWriteBuffer + strlen((char *) pcWriteBuffer), "\r\n StromPi-Mode: %s \r\n", temp_message);

	sprintf((char *) pcWriteBuffer + strlen((char *) pcWriteBuffer), "\r\n Raspberry Pi Shutdown: %s ",
			state_en_dis[shutdown_enable]);

	sprintf((char *) pcWriteBuffer + strlen((char *) pcWriteBuffer), "\r\n  Shutdown-Timer: %d seconds", shutdown_time);

	sprintf((char *) pcWriteBuffer + strlen((char *) pcWriteBuffer), "\r\n\r\n Powerfail Warning: %s ",
			state_en_dis[warning_enable]);

	sprintf((char *) pcWriteBuffer + strlen((char *) pcWriteBuffer), "\r\n\r\n Serial-Less Mode: %s ",
			state_en_dis[serialLessMode]);

	sprintf((char *) pcWriteBuffer + strlen((char *) pcWriteBuffer), "\r\n\r\n Power Save Mode: %s ",
			state_en_dis[powersave_enable]);

	sprintf((char *) pcWriteBuffer + strlen((char *) pcWriteBuffer), "\r\n\r\n Power-Off Mode: %s ",
			state_en_dis[poweroff_enable]);

	sprintf((char *) pcWriteBuffer + strlen((char *) pcWriteBuffer), "\r\n\r\n Battery-Level Shutdown: %s",
				batlevel_msg[batLevel_shutdown]);


	sprintf((char *) pcWriteBuffer + strlen((char *) pcWriteBuffer), "\r\n\r\n Powerfailure-Counter: %d",
			powerfailure_counter);

	sprintf((char *) pcWriteBuffer + strlen((char *) pcWriteBuffer), "\r\n\r\n PowerOn-Button: %s ",
			state_en_dis[powerOnButton_enable]);

	sprintf((char *) pcWriteBuffer + strlen((char *) pcWriteBuffer), "\r\n  PowerOn-Button-Timer: %d seconds",
			powerOnButton_time);

	sprintf((char *) pcWriteBuffer + strlen((char *) pcWriteBuffer), "\r\n\r\n FirmwareVersion: %s", firmwareVersion);

	/* There is no more data to return after this single string, so return
	 pdFALSE. */
	return pdFALSE;
}

/*-----------------------------------------------------------*/

/*** prvShowAlarm
 * This command is similar to the Show-Status Command but shows only the configuration of the configured Alarms
 *
 * ***/

static portBASE_TYPE prvShowAlarm(char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString)
{
	(void) pcCommandString;
	configASSERT(pcWriteBuffer);

	/* This function assumes the buffer length is adequate. */
	(void) xWriteBufferLen;

	RTC_TimeTypeDef stimestructureget;
	RTC_DateTypeDef sdatestructureget;

	HAL_RTC_GetTime(&hrtc, &stimestructureget, RTC_FORMAT_BIN);
	HAL_RTC_GetDate(&hrtc, &sdatestructureget, RTC_FORMAT_BIN);

	sprintf((char *) pcWriteBuffer, "\r\n Time: %02d:%02d:%02d", stimestructureget.Hours, stimestructureget.Minutes, stimestructureget.Seconds);

	char temp_message[21];

	sprintf((char *) pcWriteBuffer + strlen((char *) pcWriteBuffer), "\r\n Date: %s %02d.%02d.20%02d\r\n", getweekday(sdatestructureget.WeekDay),
			sdatestructureget.Date, sdatestructureget.Month, sdatestructureget.Year);


	sprintf((char *) pcWriteBuffer + strlen((char *) pcWriteBuffer), "\r\n WakeUp-Alarm: %s ",
			state_en_dis[alarm_enable]);

	if (wakeup_time_enable == 1)
		strcpy(temp_message, "Minute Wakeup Alarm");
	else if (alarmTime == 1)
		strcpy(temp_message, "Time-Alarm");
	else if (alarmDate == 1)
		strcpy(temp_message, "Date-Alarm");
	else if (alarmWeekDay == 1)
		strcpy(temp_message, "Weekday-Alarm");

	sprintf((char *) pcWriteBuffer + strlen((char *) pcWriteBuffer), "\r\n  Alarm-Mode: %s ", temp_message);

	sprintf((char *) pcWriteBuffer + strlen((char *) pcWriteBuffer), "\r\n  Alarm-Time: %02d:%02d", alarm_hour, alarm_min);

	sprintf((char *) pcWriteBuffer + strlen((char *) pcWriteBuffer), "\r\n  Alarm-Date: %02d.%02d", alarm_day, alarm_month);


	sprintf((char *) pcWriteBuffer + strlen((char *) pcWriteBuffer), "  \r\n  Minute Wakeup Time: %d ", wakeup_time);
	sprintf((char *) pcWriteBuffer + strlen((char *) pcWriteBuffer), "minutes");


	if (wakeup_time_enable == 1)
	{
		sprintf((char *) pcWriteBuffer + strlen((char *) pcWriteBuffer), "  \r\n  Minute Wakeup Time: %d ", wakeup_time);
		sprintf((char *) pcWriteBuffer + strlen((char *) pcWriteBuffer), "minutes");
	}

	sprintf((char *) pcWriteBuffer + strlen((char *) pcWriteBuffer), "\r\n  Alarm-Weekday: %s ", getweekday(alarm_weekday));

	sprintf((char *) pcWriteBuffer + strlen((char *) pcWriteBuffer), "\r\n  Weekend Wake-Up: %s \r\n ",
			state_en_dis[wakeupweekend_enable]);

	sprintf((char *) pcWriteBuffer + strlen((char *) pcWriteBuffer), "\r\n PowerOff-Alarm: %s ",
			state_en_dis[alarmPoweroff]);

	sprintf((char *) pcWriteBuffer + strlen((char *) pcWriteBuffer), "\r\n  PowerOff-Alarm-Time: %02d:%02d\r\n", alarm_hour_off, alarm_min_off);

	sprintf((char *) pcWriteBuffer + strlen((char *) pcWriteBuffer), "\r\n Interval-Alarm: %s ",
			state_en_dis[alarmInterval]);

	sprintf((char *) pcWriteBuffer + strlen((char *) pcWriteBuffer), "\r\n  Interval-Alarm-OnTime: %d minutes\r", alarmIntervalMinOn);

	sprintf((char *) pcWriteBuffer + strlen((char *) pcWriteBuffer), "\r\n  Interval-Alarm-OffTime: %d minutes\r\n", alarmIntervalMinOff);

	/* There is no more data to return after this single string, so return
	 pdFALSE. */
	return pdFALSE;
}

/*-----------------------------------------------------------*/

/*** ascii2int
 * This is a help function to convert the input of the user into the correct format for storing into the associated variables
 *
 * ***/

int ascii2int(const char* s)
{
	int i = 0;
	while (*s != 0)
	{
		if (*s < '0' || *s > '9')
		{
			return 0;
		}

		i = i * 10 + (*s - '0');
		s++;
	}
	return i;
}

/*-----------------------------------------------------------*/

/*** HAL_UART_RxCpltCallback
 * This is the STM32 Hal UART Interrupt Callback
 *
 * ***/

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{

	rx_ready = 1;
}

