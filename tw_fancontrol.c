/*
 * AVR Two/Three-Wire (TW) Fan Controller
 *
 * Controls an H-Bridge using PWM
 *
 * Grounding an IN pin will trigger an increase in the forward or
 * reverse duty cycle. Tuning the max duty cycle is done with the
 * trimpot connected to an ADC pin.
 *
 * In Two-Wire Mode:
 * Two-wire fans are bi-directional and can be controlled through PWM.
 * An H-Bridge is used to control the fan direction, and PWM is applied
 * at each H-Bridge half to control the duty cycle of the fan.
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
#define ADC_MAX		(1024)
#define ADC_MODE_THRESH	(ADC_MAX * 8/10)
/* Normalization granularity */
#define ADJ_ADC_GRAN	(16)

/* Output compare registers */
#define FWD_OC OCR1B
#define REV_OC OCR0B

enum MODE {
	TWO_WIRE,
	THREE_WIRE,
};

volatile bool irq_seen;
volatile int pwm_scaler;
volatile int prev_adj = ADC_MAX - 1;

static inline void debounce(void)
{
#ifndef DEBUG_SIMULATOR
	_delay_ms(100);
#endif
}

static inline unsigned int read_adc(void)
{
	unsigned int ret;

	/* Must read ADCL first */
	ret = ADCL;
	ret |= ADCH << 8;
	return ret;
}

/*
 * ADSC is cleared when complete
 */
static inline bool adc_busy(void)
{
	return ADCSRA & _BV(ADSC);
}

/*
 * Normalizes adc values to use more convenient scaling factors
 */
static int normalize_adc(const int raw, const int gran, const int max)
{
	int i;
	for (i = 0; i < max; i += gran) {
		if (raw < i + gran)
			return i;
	}

	/* Unreachable */
	return raw;
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
 * Poll until the conversion done interrupt fires
 */
static int get_adj_adc(void)
{
	int delay = 0;
	int ret = -1;
	while (delay < 10 && adc_busy()) {
#ifndef DEBUG_SIMULATOR
		_delay_ms(1);
		delay++;
#endif
	}

	cli();
	if (!adc_busy())
		ret = read_adc();
	stop_adj_adc();
	sei();
	return ret;
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
 * {COM1M1,COM1M0} = 'b10 - Clear on compare match. Set at 0
 * {COM1M1,COM1M0} = 'b11 - Set on compare match. Clear on 0
 * {COM0M1,COM0M0} = 'b10 - Clear on compare match when up counting
 *                          Set on compare match when down counting
 * {COM0M1,COM0M0} = 'b11 - Set on compare match when up counting
 *                          Clear on compare match when down counting
 */
static void setup_compare_params(const enum MODE mode, const bool fwd)
{
	/*
	 * In Two-Wire mode, we use Counter 0 and 1 to regulate the
	 * reverse and forward PWM behavior.
	 *
	 * In Three-Wire mode, The 'Forward' Pin is used as a hard
	 * enable and the 'Reverse' Pin is used for the PWM. Counter 0
	 * is mapped to the 'Reverse'/PWM_OUT pin, so only enable
	 * Counter 0 for this operation.
	 */
	if (fwd && mode == TWO_WIRE)
		GTCCR |= FWD_MASK;
	else /* !fwd && TWO_WIRE || fwd && THREE_WIRE */
		TCCR0A |= REV_MASK;
}

static inline void clear_compare_params(void)
{
	GTCCR &= ~FWD_MASK;
	TCCR0A &= ~REV_MASK;
}

/* Only for Three-Wire mode */
static inline void set_out_fwd(void)
{
#if OUT_FWD_ACTIVE_HIGH
	PORTB |= OUT_FWD;
#else
	PORTB &= ~OUT_FWD;
#endif
}

/*
 * Clears outputs to the motor driver by setting their values to the
 * driver's 'off' logic level.
 */
static inline void clear_outputs(void)
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
 * Set TCNT compare match, reverse, and PWM frequency parameters
 * As soon as the frequency parameters are set, the counters will start
 * counting. They are stopped by stop_count
 */
static void start_count_and_enable(const enum MODE mode, const bool fwd)
{
	if (mode == THREE_WIRE)
		set_out_fwd();

	/* Set up counter prescaler and start count */
#ifdef DEBUG_SIMULATOR
	/* 1/1 */
	if (fwd && mode == TWO_WIRE)
		TCCR1 |= _BV(CS10);
	else /* !fwd && TWO_WIRE || fwd && THREE_WIRE */
		TCCR0B |= _BV(CS00);
#else
	/* 1/64 */
	if (fwd && mode == TWO_WIRE)
		TCCR1 |= _BV(CS12) | _BV(CS11) | _BV(CS10);
	else /* !fwd && TWO_WIRE || fwd && THREE_WIRE */
		TCCR0B |= _BV(CS01) | _BV(CS00);
#endif
}

/*
 * Sets motor driver 'off' logic level and stops the counters
 */
static void stop_count_and_disable(const enum MODE mode)
{
	clear_outputs();

	/* Stop forward counter */
	if (mode == TWO_WIRE)
		TCCR1 &= ~(_BV(CS13) | _BV(CS12) | _BV(CS11) | _BV(CS10));

	/* Stop reverse counter (forward in Three-Wire mode) */
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
static inline void setup_tcnt0_pwm(void)
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
static inline void setup_tcnt1_pwm(void)
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
static void update_pwm(const enum MODE mode, const bool fwd)
{
	int duty_cycle, scaler, adj;
	bool was_stopped = is_stopped();

	/* Blocks until conversion done or timeout */
	adj = get_adj_adc();
	if (adj > 0) {
		/* Normalize to rounder values */
		adj = normalize_adc(adj, ADJ_ADC_GRAN, ADC_MAX);
	} else {
		/* Timed out. Use previous scaler */
		adj = prev_adj;
	}

	/*
	 * Calculate new duty cycle based on the scaler, so that we can
	 * always return to 0. In two-wire mode, the scaler and duty
	 * cycle are positive when in the forward direction and negative
	 * when in the reverse direction. In three-wire mode, the scaler
	 * and duty cycle are always positive.
	 */
	cli();
	scaler = pwm_scaler + (fwd ? 1 : -1);
	if (mode == THREE_WIRE && scaler < 0)
		scaler = 0;
	duty_cycle = scaler * SCALER_INCR * adj / ADC_MAX;
	prev_adj = adj;
	sei();

	/* Catch PWM overflow */
	if (abs(duty_cycle) > 0xFF)
		return;

	/* Increment the scaler and set PWM compare match values */
	cli();
	pwm_scaler = scaler;
	if (mode == TWO_WIRE) {
		if (pwm_scaler >= 0) {
			FWD_OC = duty_cycle;
			REV_OC = 0;
		} else {
			FWD_OC = 0;
			REV_OC = -duty_cycle;
		}
	} else {
		/* Three-Wire */
		REV_OC = duty_cycle;
	}

	/* Detect stop->start and start->stop transitions */
	if (was_stopped) {
		setup_compare_params(mode, fwd);
		start_count_and_enable(mode, pwm_scaler > 0);
	} else if (is_stopped()) {
		stop_count_and_disable(mode);
		clear_compare_params();
	}

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

static void handle_irq(const enum MODE mode)
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
		update_pwm(mode, 1);
	else if (rev && get_rev())
		update_pwm(mode, 0);

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

static inline void setup_mode_adc(void)
{
	/* Set VREF = VCC */
	ADMUX = 0;

	/* Set input (ADC2) */
	ADMUX |= _BV(MUX1);
	DIDR0 |= _BV(ADC2D);

	/* Set internal pull-up */
	PORTB |= OUT_FWD;
}

/*
 * Start ADC conversion without the interrupt enabled. At this point, we
 * can just rely on polling the ADC. We would normally want to set up
 * the interrupt and sleep, but this is only done on startup so the
 * overhead is negligible. Remember that we are probably going to be
 * powered for long periods of time, but sleeping between actions.
 */
static inline void start_mode_adc(void)
{
	ADCSRA |= _BV(ADEN) | _BV(ADSC);
}

static inline void release_mode_adc(void)
{
	ADMUX &= ~_BV(MUX1);
	DIDR0 &= ~_BV(ADC2D);
	PORTB &= ~OUT_FWD;
}

/*
 * Mode is selected by soldering the jumper that connects OUT_FWD to a
 * resistor to ground. We temporarily use this pin as an input. With the
 * internal pull-up resistor, the ADC will detect a voltage divider and
 * this selects Three-Wire mode. If the solder jumper is not jumped, the
 * ADC will read near MAX due to the pull-up, and Two-Wire mode is
 * selected.
 *
 * In Two-Wire mode, this pin is doing PWM and won't be affected by the
 * external resistor, because the jumper connecting them is not
 * soldered. In Three-Wire mode, this pin is used as a hard enable, and
 * won't be affected by the weak external resistor.
 */
static enum MODE early_detect_mode(void) { enum MODE mode = TWO_WIRE;

	setup_mode_adc();
	start_mode_adc();
	while (adc_busy())
		;

	if (read_adc() < ADC_MODE_THRESH)
		mode = THREE_WIRE;

	release_mode_adc();
	return mode;
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
	enum MODE mode = early_detect_mode();

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
			handle_irq(mode);
		}
		relax();
	}
}
