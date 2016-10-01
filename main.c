/*
 * AVR 2-wire Fan Controller
 *
 * Copyright (c) 2016, Jon Derrick
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
 */

//#define DEBUG_SIMULATOR

#include <stdlib.h>
#include <stdbool.h>

#define F_CPU 1000000UL
#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/sleep.h>
#include <util/delay.h>

#define CYCLE_INCR 15
#define OUT_HIGH 0

#define IN_UP		_BV(PINB3)
#define IN_DOWN		_BV(PINB4)
#define OUT_UP		_BV(PINB1)
#define OUT_DOWN	_BV(PINB0)
#define get_up() (~PINB & IN_UP)
#define get_down() (~PINB & IN_DOWN)

#if 0
	bool is_on = true;
	#define OFF() do { PORTB |= (OUT_UP | OUT_DOWN); } while(0)
	#define ON() do { PORTB &= ~(OUT_UP | OUT_DOWN); } while(0)
	#define INV() do { if (is_on) OFF(); else ON(); is_on = !is_on; } while(0)
#endif

volatile bool irq_seen;
volatile int pwm_dc;

static inline void debounce(void)
{
#ifndef DEBUG_SIMULATOR
	_delay_ms(100);
#endif
}

static void start_count(bool up)
{
#if OUT_HIGH
	/* Set TCNT reverse parameters. Cleared by stop_count */
	if (up)
		TCCR0A |= _BV(COM0B1);
	else
		TCCR0A |= _BV(COM0A1);
#else
	if (up)
		TCCR0A |= _BV(COM0B1) | _BV(COM0B0);
	else
		TCCR0A |= _BV(COM0B1) | _BV(COM0B0) | _BV(COM0A1) | _BV(COM0A0);
#endif

#ifdef DEBUG_SIMULATOR
	/* 1/1 prescaler */
	TCCR0B |= _BV(CS00);
#else
	/* 1/64 prescaler */
	TCCR0B |= _BV(CS01) | _BV(CS00);
#endif
}

static void clear_out()
{
#if OUT_HIGH
	/* Clear */
	PORTB &= ~(OUT_UP | OUT_DOWN);
#else
	/* Set */
	PORTB |= (OUT_UP | OUT_DOWN);
#endif
}

static void stop_count(void)
{
	clear_out();
	TCCR0A &= ~(_BV(COM0B1) | _BV(COM0A1));
	TCCR0B &= ~(_BV(CS02) | _BV(CS01) | _BV(CS00));
}

static void setup_pwm(void)
{
	/* Phase correct, max 0xFF */
	TCCR0A |= _BV(WGM00);
	clear_out();
}

static void update_pwm(bool up)
{
	int duty_cycle = up ? CYCLE_INCR : -CYCLE_INCR;
	bool stopped = pwm_dc == 0;

	cli();
	/* Tried to do a goto here but it didn't work correctly */
	if (abs(pwm_dc + duty_cycle) > 0xFF) {
		sei();
		return;
	}

	pwm_dc += duty_cycle;
	if (pwm_dc >= 0) {
		OCR0A = pwm_dc;
		OCR0B = pwm_dc;
	} else {
		OCR0A = -pwm_dc;
		OCR0B = -pwm_dc;
	}

	if (stopped)
		start_count(pwm_dc > 0);
	else if (pwm_dc == 0)
		stop_count();

	sei();
}

ISR(PCINT0_vect)
{
	/* Clear pin change int enable and schedule debouncing work */
	GIMSK &= ~_BV(PCIE);
	irq_seen = true;
	sei();
}

static void setup_irq(void)
{
	/* Setup the pin change interrupt mask, flag, and enable */
	cli();
	PCMSK |= IN_UP | IN_DOWN;
	GIMSK |= _BV(PCIE);
	sei();
}

static void handle_irq(void)
{
	bool up = get_up();
	bool down = get_down();

	debounce();
	if (up && get_up())
		update_pwm(1);
	else if (down && get_down())
		update_pwm(0);

	GIMSK |= _BV(PCIE);
	irq_seen = false;

	/* Don't allow 'hold' detection when switching directions */
	if (pwm_dc == 0) {
		_delay_ms(2000);
		return;
	}

	debounce();
	if (get_up() || get_down())
		irq_seen = true;
}

static void setup_portb(void)
{
	/* Set input pull-ups */
	PORTB = IN_UP | IN_DOWN;

	/* Set outputs */
	PINB = 0;
	DDRB |= OUT_UP | OUT_DOWN;
}

static void setup_sleep(int mode)
{
	set_sleep_mode(mode);
}

static inline void relax(void)
{
	cli();
	sleep_enable();
	sei();
	sleep_cpu(); /* Blocks until done sleeping */
	sleep_disable();
}

int main(void)
{
	setup_portb();
	setup_pwm();
	setup_irq();

	/* Setup sleep and relax */
	setup_sleep(SLEEP_MODE_PWR_DOWN);
	while (1) {
		if (irq_seen) {
			handle_irq();
		}
		//relax();
	}
}
