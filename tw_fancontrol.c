/*
 * AVR Two/Three-Wire (TW) Fan Controller
 *
 * Controls an h-bridge using PWM
 *
 * Grounding an IN pin will trigger an increase in the forward or
 * reverse duty cycle. Tuning the max duty cycle is done with the
 * trimpot connected to an ADC pin.
 *
 * In Two-Wire Mode:
 * Two-wire fans are bi-directional and can be controlled through PWM.
 * An H-bridge is used to control the fan direction, and PWM is applied
 * at each H-bridge half to control the duty cycle of the fan.
 *
 * In Three-Wire Mode:
 * Three-wire fans use two wires for Power and Ground, and a third for
 * PWM. TW Fan Controller uses the first PWM OUT to the H-Bridge as a
 * hard enable/disable for the FAN, and uses the second PWM OUT as the
 * control to the PWM wire. The fan cannot be reversed in this
 * configuration, but will still operate up to max and down to 0.
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

/*
 * Output PWM pin logic level behavior for motor driver. In case you end
 * up on an inverted pin or using a motor driver with a different truth
 * table.
 */
#define OUT_FWD_ACTIVE_HIGH 1
#define OUT_REV_ACTIVE_HIGH 1

#if OUT_FWD_ACTIVE_HIGH
#define FWD_MASK _BV(COM1B1)
#else
#define FWD_MASK (_BV(COM1B1) | _BV(COM1B0))
#endif

#if OUT_REV_ACTIVE_HIGH
#define REV_MASK _BV(COM0A1)
#else
#define REV_MASK (_BV(COM0A1) | _BV(COM0A0))
#endif

/* Alias the pins for convenience */
#define IN_FWD		_BV(PINB3)
#define IN_REV		_BV(PINB1)
#define OUT_FWD		_BV(PINB4)
#define OUT_REV		_BV(PINB0)
#define get_fwd()	(~PINB & IN_FWD)
#define get_rev()	(~PINB & IN_REV)

/* ADC specific functions */
#define IN_ADJ		_BV(PINB2)
#define IN_MODE		_BV(PINB4)
#define ADC_ADJ		_BV(ADC1)
#define ADC_MODE	_BV(ADC2)
#define ADC_MAX		(1024) //FIXME: find a platform macro

/* Output compare registers */
#define FWD_OC OCR1B
#define REV_OC OCR0B

volatile bool irq_seen;
volatile int pwm_scaler;
volatile int prev_adj = ADC_MAX - 1;

static inline void debounce(void)
{
#ifndef DEBUG_SIMULATOR
	_delay_ms(100);
#endif
}

/*
 * Enable the ADC and start the conversion. The ADC interrupt is used to
 * detect when it has finished.
 */
static inline void start_adj_adc(void)
{
	ADCSRA |= _BV(ADEN) | _BV(ADSC) | _BV(ADIE);
}

/*
 * Clear ADC enable and the conversion complete interrupt enable
 */
static inline void stop_adj_adc(void)
{
	ADCSRA &= ~(_BV(ADEN) | _BV(ADIE));
}

ISR(ADC_vect)
{
	stop_adj_adc();
	sei();
}

/*
 * Set ADSC to start converting and it will be cleared when complete
 */
static inline bool adj_adc_busy(void)
{
	return ADCSRA & _BV(ADSC);
}

/*
 * Poll until the conversion done interrupt fires
 */
static int get_adj_adc(void)
{
	int delay = 0;
	int ret = -1;
	while (delay < 10 && adj_adc_busy()) {
		_delay_ms(1);
		delay++;
	}

	cli();
	if (!adj_adc_busy()) {
		/* Must read ADCL first */
		ret = ADCL;
		ret |= ADCH << 8;
	}
	stop_adj_adc();
	sei();
	return ret;
}

/*
 * Normalizes adc values to use more convenient scaling factors
 */
static int normalize_adj_adc(const int adj, const int incr)
{
	int i;
	for (i = 0; i < ADC_MAX; i += incr) {
		if (adj < i + incr)
			return i;
	}

	/* Unreachable */
	return adj;
}

static void setup_adj_adc(void)
{
	/* Set VREF = VCC */
	ADMUX = 0;

	/* Set input (ADC1) */
	ADMUX |= _BV(MUX0);
	DIDR0 |= _BV(ADC1D);
}

/*
 * Set TCNT compare and reverse parameters
 *
 * {COM1M1,COM1M0} = 'b10 - Clear on compare match. Set at 0
 * {COM1M1,COM1M0} = 'b11 - Set on compare match. Clear on 0
 * {COM0M1,COM0M0} = 'b10 - Clear on compare match when up counting
 *                          Set on compare match when down counting
 * {COM0M1,COM0M0} = 'b11 - Set on compare match when up counting
 *                          Clear on compare match when down counting
 */
static void start_count(const bool fwd)
{
	/*
	 * Set up compare parameters
 	 * Cleared by stop_count
 	 */
	if (fwd)
		GTCCR |= FWD_MASK;
	else
		TCCR0A |= REV_MASK;

	/* Set up counter prescaler and start count */
#ifdef DEBUG_SIMULATOR
	/* 1/1 */
	if (fwd)
		TCCR1 |= _BV(CS10);
	else
		TCCR0B |= _BV(CS00);
#else
	/* 1/64 */
	if (fwd)
		TCCR1 |= _BV(CS12) | _BV(CS11) | _BV(CS10);
	else
		TCCR0B |= _BV(CS01) | _BV(CS00);
#endif
}

/*
 * Clears outputs to the motor driver by setting their values to the
 * driver's 'off' logic level.
 */
static void clear_outputs()
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
	clear_outputs();

	/* Clear forward counter and compare parameters */
	GTCCR &= ~FWD_MASK;
	TCCR1 &= ~(_BV(CS13) | _BV(CS12) | _BV(CS11) | _BV(CS10));

	/* Clear reverse counter and compare parameters */
	TCCR0A &= ~REV_MASK;
	TCCR0B &= ~(_BV(CS02) | _BV(CS01) | _BV(CS00));
}

static inline bool is_stopped(void)
{
	return pwm_scaler == 0;
}

/*
 * Sets up Counter 0 in phase-correct PWM mode with a 'TOP' value of
 * 0xFF. In this mode, the counter will count up to TOP and reverse
 * direction, counting down. The duty cycle of the OCn pins are
 * determined by the values of the compare match registers (OCRnM) and
 * the action to perform on compare match (TCCR0A:{COMnM1,COMnM0}).
 */
static void setup_tcnt0_pwm(void)
{
	/* Phase correct, max 0xFF */
	TCCR0A |= _BV(WGM00);
}

/*
 * Sets up Counter 1 PWM mode with a 'TOP' value of 0xFF. In this mode,
 * the counter will count up to TOP and clear itself. The duty cycle of
 * the OCn pins are determined by the values of the compare match
 * registers (OCRnM) and the action to perform on TCNT reset (OCRnC),
 * and the action to perform on compare match GTCCR:{COMnM1,COMnM0}).
 */
static void setup_tcnt1_pwm(void)
{
	/* Count to OCR1C and reset */
	GTCCR |= _BV(PWM1B);
	OCR1C = 0xFF;
}

static void setup_pwm(void)
{
	setup_tcnt0_pwm();
	setup_tcnt1_pwm();
	clear_outputs();
}

/*
 * Increments the PWM in whichever direction is specified by the input
 * pin change event.
 */
static void update_pwm(const bool fwd)
{
	const int adc_incr = 16;
	int duty_cycle, scaler, adj;
	bool was_stopped = is_stopped();

	/* Blocks until conversion done or timeout */
	adj = get_adj_adc();
	if (adj > 0) {
		/* Normalize to rounder values */
		adj = normalize_adj_adc(adj, adc_incr);
	} else {
		/* Timed out. Use previous scaler */
		adj = prev_adj;
	}

	/*
	 * Calculate new duty cycle based on the scaler, so that we can
	 * always return to 0. The scaler and duty cycle are positive
	 * when in the forward direction and negative when in the
	 * reverse direction.
	 */
	cli();
	scaler = pwm_scaler + (fwd ? 1 : -1);
	duty_cycle = scaler * SCALER_INCR * adj / ADC_MAX;
	prev_adj = adj;
	sei();

	/* Catch PWM overflow */
	if (abs(duty_cycle) > 0xFF)
		return;

	/* Increment the scaler and set PWM compare match values */
	cli();
	pwm_scaler = scaler;
	if (pwm_scaler >= 0) {
		FWD_OC = duty_cycle;
		REV_OC = 0;
	} else {
		FWD_OC = 0;
		REV_OC = -duty_cycle;
	}

	/* Detect stop->start and start->stop transitions */
	if (was_stopped)
		start_count(pwm_scaler > 0);
	else if (is_stopped())
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

	/* The ADC takes a few cycles, so start it early */
	start_adj_adc();

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
	if (is_stopped()) {
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

#if 0 /* For 2w/3w mode selection */
static void setup_mode_adc(void)
{
	/* Set VREF = VCC */
	ADMUX = 0;

	/* Set input (ADC2) */
	ADMUX |= _BV(MUX1);
	DIDR0 |= _BV(ADC2D);
}

static void release_mode_adc(void)
{
	ADMUX &= ~_BV(MUX1);
	DIDR0 &= ~_BV(ADC2D);
}
#endif

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
	setup_adj_adc();
	setup_irq();

	/* The uC will remain in a low power mode until a pin change
	 * interrupt is thrown. After servicing the interrupt and the
	 * scheduled work, the uC will power back down to the low power
	 * mode.
	 */
	set_sleep_mode(SLEEP_MODE_PWR_DOWN);
	while (1) {
		if (irq_seen) {
			handle_irq();
		}
		relax();
	}
}
