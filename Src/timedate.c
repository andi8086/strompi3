/*
 * timedate.c
 *
 *  Created on: Mar 28, 2021
 *      Author: andreas
 */

#include <stdint.h>

const char *weekdays[7] = {"Monday", "Tuesday", "Wednesday", "Thursday", "Friday", "Saturday", "Sunday"};

const char *getweekday(uint8_t idx)
{
	if (idx > 6) {
		return "";
	}

	return weekdays[idx];
}
