/*
 * This file is part of the HAL project, inline library above libopencm3.
 *
 * This library is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with this library.  If not, see <http://www.gnu.org/licenses/>.
 */
/** @defgroup DELAY_module DELAY module
 *
 * @brief Spin-waiting blocking API
 *
 * @ingroup modules
 *
 * LGPL License Terms @ref lgpl_license
 */
#ifndef HAL_DELAY_H_INCLUDED
#define HAL_DELAY_H_INCLUDED


/**@{*/

/*****************************************************************************/
/* API definitions                                                           */
/*****************************************************************************/

/*****************************************************************************/
/* API Functions                                                             */
/*****************************************************************************/

// BEGIN_DECLS

/*---------------------------------------------------------------------------*/
/** @brief Spin-wait delay, spinning specified amount of processor cycles
 *
 * @note this function can be used for delays of max 2500000 cycles.
 * For larger delays, please consider using timers or other waiting techniques.
 *
 * @param[in] cycles Cycles count need to spent in spin-wait
 */
static void delay_cycles(const int64_t cycles);

/*---------------------------------------------------------------------------*/
/** @brief Spin-wait delay, spinning specified amount of microseconds
 *
 * @note this function can be used for delays max 25 sec @ 168MHz CPU clock, or
 * max 525 sec @ 8MHz CPU clock! For larger delays, please consider using
 * timers or other waiting techniques.
 *
 * @param[in] us Microseconds needed to spin wait.
 * @param[in] cpufreq Current CPU frequency in Hz
 */
static void delay_us(uint32_t us);

/*---------------------------------------------------------------------------*/
/** @brief Spin-wait delay, spinning specified amount of microseconds
 *
 * @note this function can be used for delays max 25 sec @ 168MHz CPU clock, or
 * max 525 sec @ 8MHz CPU clock! For larger delays, please consider using
 * timers or other waiting techniques.
 *
 * @param[in] ms Milliseconds needed to spin wait.
 * @param[in] cpufreq Current CPU frequency in Hz
 */
static void delay_ms(uint32_t ms, uint64_t cpufreq);

// END_DECLS

/**@}*/

/*****************************************************************************/
/* Architecture dependent implementations                                    */
/*****************************************************************************/


#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-function"

static void _delay_3t(uint32_t cycles) __attribute__((naked));

/* 3 Tcyc per tick, 4Tcyc call/ret, 1Tcyc hidden reg assignment */
static void _delay_3t(uint32_t cycles)
{
	asm __volatile__ (
        ".syntax unified\n"
		"1: \n"
		"	subs %[cyc],#1 \n"	/* 1Tck */
		"	bne 1b \n"		/* 2Tck */
		"	bx lr \n"
		".syntax divided\n"
		: /* No output */
		: [cyc] "r" (cycles)
		: /* No memory */
	);
}


void delay_cycles(const int64_t cycles)
{
	if (cycles <= 0)
		return;

	switch (cycles % 3) {
	default:
	case 0: break;
	case 1: asm __volatile__ ("nop"); break;
	case 2: asm __volatile__ ("nop\nnop"); break;
	}

	if (cycles > 3)
		_delay_3t((uint32_t)(cycles / 3));
	else /* same delay as the function call */
		asm __volatile__ ("nop\nnop\nnop\nnop\nnop\nnop\n");
}


/* max 25 sec @ 168MHz! */
/* max 525 sec @ 8MHz! */
void delay_us(uint32_t us)
{
	if (us == 0)
		return;

	delay_cycles(us * CLOCK_MHZ / 1000000 - 6);
}




/* max 25 sec @ 168MHz! */
/* max 525 sec @ 8MHz! */
void delay_ms(uint32_t ms, uint64_t cpufreq)
{
	if (ms == 0)
		return;

	delay_cycles(ms * cpufreq / 1000 - 6);
}

#pragma GCC diagnostic pop


#endif /* HAL_DELAY_H_INCLUDED */
