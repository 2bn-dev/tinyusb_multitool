/*! \file
 * \brief Contains convenience debug output function that is used when compiled in debug mode can be overridden by defining _DBG
 *
 */

#ifndef TINYUSB_MULTITOOL_DEBUG_H
#define TINYUSB_MULTITOOL_DEBUG_H

#include <stdio.h>

#ifdef PICO_BUILD
#include "pico/time.h"
#else
void get_absolute_time() return 0;
#endif

#ifndef NDEBUG
#ifndef _DBG
#define _DBG(fmt, ...) \
	do{ fprintf(stdout, "%llu [%s:%d](%s) - " fmt "\r\n", \
		get_absolute_time(), __FILE__, __LINE__, __func__, ##__VA_ARGS__); } while(0)
#endif
#else
#define _DBG(fmt, ...)
#endif

#endif
