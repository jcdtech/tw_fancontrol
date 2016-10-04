/*
 * AVR 2-wire Fan Controller
 *
 * Controls an h-bridge using PWM
 * Grounding an IN pin will trigger an increase in the forward or
 * reverse duty cycle. Tuning the max duty cycle is done with the
 * trimpot connected to an ADC pin.
 *
 * This circuit may go very long times between use, so the
 * microcontroller must use very little power when not driving the fan.
 * The microcontroller's sleep mode is enabled when not changing state,
 * and coming out of sleep mode is entirely interrupt driver by the pin
 * change interrupt. PWM is accomplished in hardware and the ADC is only
 * polled when changing the PWM duty cycle.
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
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA
 * 02110-1301, USA.
 */

#include <stdlib.h>
#include <stdbool.h>

#define F_CPU 1000000UL
#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/sleep.h>
#include <util/delay.h>

//#define DEBUG_SIMULATOR

/* The base PWM increment value relative to COUNT_TOP (0xFF) */
#define SCALER_INCR 15

/* Output PWM pin logic level behavior for motor driver */
#define OUT_FWD_ACTIVE_HIGH 1
#define OUT_REV_ACTIVE_HIGH 1

#if OUT_FWD_ACTIVE_HIGH
#define FWD_MASK _BV(COM0B1)
#else
#define FWD_MASK (_BV(COM0B1) | _BV(COM0B0))
#endif

#if OUT_REV_ACTIVE_HIGH
#define REV_MASK _BV(COM0A1)
#else
#define REV_MASK (_BV(COM0A1) | _BV(COM0A0))
#endif

/* Alias the pins for convenience */
#define IN_FWD		_BV(PINB3)
#define IN_REV		_BV(PINB4)
#define OUT_FWD		_BV(PINB1)
#define OUT_REV		_BV(PINB0)
#define get_fwd()	(~PINB & IN_FWD)
#define get_rev()	(~PINB & IN_REV)
//FIXME
#define IN_ADJ		_BV(PINB2)
#define get_adj()	(ADC_MAX())
#define ADC_MAX()	(1024)

/* DEBUG CODE */
#if 0
	bool is_on = true;
	#define OFF() do { PORTB |= (OUT_FWD | OUT_REV); } while(0)
	#define ON() do { PORTB &= ~(OUT_FWD | OUT_REV); } while(0)
	#define INV() do { if (is_on) OFF(); else ON(); is_on = !is_on; } while(0)
#endif

volatile bool irq_seen;
volatile int pwm_scaler;
#define stopped() (pwm_scaler == 0)

static inline void debounce(void)
{
#ifndef DEBUG_SIMULATOR
	_delay_ms(100);
#endif
}

/*
 * Set TCNT compare and reverse parameters
 * Cleared by stop_count
 *
 * {COMnM1,COMnM0} = 'b10 - Clear on compare match when up counting
 * {COMnM1,COMnM0} = 'b11 - Set on compare match when up counting
 */
static void start_count(bool fwd)
{
	/* Set up compare parameters */
	if (fwd)
		TCCR0A |= FWD_MASK;
	else
		TCCR0A |= REV_MASK;

	/* Set up counter prescaler and start count */
#ifdef DEBUG_SIMULATOR
	/* 1/1 */
	TCCR0B |= _BV(CS00);
#else
	/* 1/64 */
	TCCR0B |= _BV(CS01) | _BV(CS00);
#endif
}

/*
 * Clears outputs to the motor driver by setting their values to the
 * driver's 'off' logic level.
 */
static void clear_out()
{
#if OUT_FWD_ACTIVE_HIGH
	PORTB &= ~OUT_FWD;
#else
	PORTB |= OUT_FWD;
#endif

#if OUT_REV_ACTIVE_HIGH
	PORTB &= ~OUT_REV;
#else
	PORTB |= OUT_REV;
#endif
}

/*
 * Sets motor driver 'off' logic level and stops the counters
 */
static void stop_count(void)
{
	clear_out();
	TCCR0A &= ~(FWD_MASK | REV_MASK);
	TCCR0B &= ~(_BV(CS02) | _BV(CS01) | _BV(CS00));
}

/*
 * Sets up Counter 0 in phase-correct PWM mode with a 'TOP' value of
 * 0xFF. In this mode, the counter will count up to TOP and reverse
 * direction, counting down. The duty cycle of the OCn pins are
 * determined by the values of the compare match registers (OCRnM) and
 * the action to perform on compare match (TCCR0A:{COMnM1,COMnM0}).
 */
static void setup_pwm(void)
{
	/* Phase correct, max 0xFF */
	TCCR0A |= _BV(WGM00);
	clear_out();
}

/*
 * Increments the PWM in whichever direction is specified by the input
 * pin change event.
 */
static void update_pwm(bool fwd)
{
	int duty_cycle, scaler;
	bool was_stopped = stopped();

	/*
	 * Calculate new duty cycle based on the scaler, so that we can
	 * always return to 0. The scaler and duty cycle are positive
	 * when in the forward direction and negative when in the
	 * reverse direction.
	 */
	cli();
	scaler = pwm_scaler + (fwd ? 1 : -1);
	duty_cycle = scaler * SCALER_INCR * get_adj() / ADC_MAX();
	sei();

	/* Catch PWM overflow */
	if (abs(duty_cycle) > 0xFF)
		return;

	/* Increment the scaler and set PWM compare match values */
	cli();
	pwm_scaler = scaler;
	if (pwm_scaler >= 0) {
		OCR0A = duty_cycle;
		OCR0B = 0;
	} else {
		OCR0A = 0;
		OCR0B = -duty_cycle;
	}

	/* Detect stop->start and start->stop transitions */
	if (was_stopped)
		start_count(pwm_scaler > 0);
	else if (stopped())
		stop_count();

	sei();
}

ISR(PCINT0_vect)
{
	/* Clear pin change irq enable and schedule debouncing work */
	GIMSK &= ~_BV(PCIE);
	irq_seen = true;
	sei();
}

static void setup_irq(void)
{
	/* Setup the pin change interrupt mask, flag, and enable */
	cli();
	PCMSK |= IN_FWD | IN_REV;
	GIMSK |= _BV(PCIE);
	sei();
}

static void handle_irq(void)
{
	bool fwd = get_fwd();
	bool rev = get_rev();

	/*
	 * The pin change interrupts need to be debounced in software to
	 * prevent multiple updates. This is done by checking the pin
	 * values before and after a fixed debounce time.
	 */
	debounce();
	if (fwd && get_fwd())
		update_pwm(1);
	else if (rev && get_rev())
		update_pwm(0);

	/* Re-enable the pin change interrupt */
	GIMSK |= _BV(PCIE);
	irq_seen = false;

	/* Stall 'button hold' behavior when switching directions */
	if (stopped()) {
		_delay_ms(2000);
		return;
	}

	/* Catch 'button hold' and reschedule isr */
	debounce();
	if (get_fwd() || get_rev())
		irq_seen = true;
}

static void setup_portb(void)
{
	/* Set input pull-ups */
	PORTB = IN_FWD | IN_REV;
	PINB = 0;

	/* Set outputs */
	DDRB |= OUT_FWD | OUT_REV;
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

	/* The uC will remain in a low power mode until a pin change
	 * interrupt is thrown. After servicing the interrupt and the
	 * scheduled work, the uC will power back down to the low power
	 * mode.
	 */
	setup_sleep(SLEEP_MODE_PWR_DOWN);
	while (1) {
		if (irq_seen) {
			handle_irq();
		}
		relax();
	}
}
