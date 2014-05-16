// -*- c++ -*-
/* DUBotics Hydra: low-level motor shaft encoder driver
 * Copyright (C) 2014 DUBotics at the University of Washington
 *
 * This program is free software; you can redistribute it and/or modify it under
 * the terms of the GNU General Public License as published by the Free Software
 * Foundation; either version 2 of the License, or (at your option) any later
 * version.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS
 * FOR A PARTICULAR PURPOSE.  See the GNU General Public License for more
 * details.
 *
 * The GNU General Public License version 2 may be found at
 * <http://www.gnu.org/licenses/gpl-2.0.html>.
 */
/**@file
 *
 * Arduino sketch implementing a driver for two-bit Gray Code-based rotary
 * encoders, with communication via I2C/TWI.
 *
 * For efficiency, this sketch makes heavy use of low-level IO --- it doesn't
 * rely on the (really quite terribly slow) `digitalRead` and `digitalWrite`
 * from the Arduino standard library, for example.
 *
 * That is to say, we're not in kindergarten anymore, kids.  Oh, and don't
 * even bother trying to replace this code (which utilizes Pin-Change
 * Interrupts) with something that uses `attachInterrupt` (which uses External
 * Interrupt Requests, which are only supported on a few pins).
 */

/** @name Pin definitions
 *
 *  `#define`s used to configure and read the I/O pins to which the encoder is
 *  attached.  Please consult the target microcontroller's datasheet when
 *  modifying these values.
 *
 *  *The fact that these values are all zeros and ones does not mean that yours
 *  will be if you decide to change the pin mapping.*  For example, if we were
 *  to
 *
 *      #define P0_PIN PIND6
 *
 *  we would also need
 *
 *      #define P0_PORT PIND
 *      #define P0_PCINT PCINT21
 *
 *      #define P0_PCIE PCIE2
 *      #define P0_PCMSK PCMSK2
 *      #define P0_PCINT_vect PCINT2_vect
 *
 * **Always** consult your microcontroller's datasheet.  Note that these
 * definitions don't handle pin-direction initialization; you'll have to modify
 * the first few lines of `setup` if you change the pins used.
 *
 *  The datasheet for the ATmega48PA/88PA/168PA/328P is available
 *  (here)[http://www.atmel.com/Images/doc8161.pdf].
 *
 * @{
 */

/** Input ports for the pins attached to the encoder outputs. */
#define P0_PORT PINB
#define P1_PORT PINB

/** Pins, on P0_PORT and P1_PORT, respectively, for the pins attached to the
    encoder outputs. */
#define P0_PIN PINB0
#define P1_PIN PINB1

/** Pin-change-interrupt pin labels for these pins. */
#define P0_PCINT PCINT0
#define P1_PCINT PCINT1

/* NOTE that each pin's respective definition in each of the following three
   pairs is closely related to the other two; the `n` in `Pn_PCIE`, `Pn_PCMSK`,
   and `Pn_PCINT_vect` is the index of a group of pins that share a common
   pin-change interrupt.
 */
/** Pin-change interrupt enable flags corresponding to the pins attached to the
    encoder outputs. */
#define P0_PCIE PCIE0
#define P1_PCIE PCIE0

/** Pin-change interrupt mask registers in which P0_PCINT and P1_PCINT appear. */
#define P0_PCMSK PCMSK0
#define P1_PCMSK PCMSK0

/** Pin-change *interrupt* names.  Don't confuse these with the above
    definitions of `Pn_PCINT` --- those are pin-specific values, while these
    names are used to define interrupt service routines for _groups_ of PCI
    pins.  */
#define P0_PCINT_vect PCINT0_vect
#define P1_PCINT_vect PCINT0_vect
/**@} */


/** Default I2C/TWI slave address.  As originally written we grab the value as
    the 7 bits ((PD7:5) >> 1) | (PC3:0).  */
#define DEFAULT_I2C_SLAVE_ADDRESS                               \
  (((PIND & (_BV(PIND7)|_BV(PIND6)|_BV(PIND5))) >> 1) |         \
   ((PINC & (_BV(PINC3)|_BV(PINC2)|_BV(PINC1)|_BV(PINC0)))))

/* Configuration constants end here. */

/** Most recent value of the encoder output pins. */
static uint8_t prev_state;

/** Rotation/value counter. */
static int16_t value;

enum
  {
    I2C_REGISTER_VALUE = 0,
    I2C_REGISTER_VALUE_WITH_RESET
  };

#define FETCH_ENCODER_STATE() ( ((P0_PORT & _BV(P0_PIN)) >> (P0_PIN - 1)) | ((P1_PORT & _BV(P1_PIN)) >> P1_PIN) )

/** Body of the interrupt-service routine. */
#define ISR_BODY                                                        \
  /* Toggle the LED. */                                                 \
  PINB |= _BV(PINB5);                                                   \
                                                                        \
  /* A clockwise-turning shaft produces the value sequence              \
                                                                        \
      A 1 1 0 1 1 0 1 1 ...                                             \
      B 0 1 1 0 1 1 0 1 ...                                             \
                                                                        \
    while a counterclockwise-turning shaft produces                     \
                                                                        \
      A 0 1 1 0 1 1 0 1 1 ...                                           \
      B 1 1 0 1 1 0 1 1 0 ...                                           \
                                                                        \
    so it's possible to construct a look-up table indexed by the bits (A₁ B₁ A₀ B₀) as follows. \
                                                                        \
      A₁ B₁     Index	Dir.                                            \
      A₀ B₀                                                             \
      -----------------------------                                     \
      1  0	11	+                                               \
      1  1                                                              \
                                                                        \
      1  1	13	+                                               \
      0  1                                                              \
                                                                        \
      0  1	6	+                                               \
      1  0                                                              \
                                                                        \
      0  1	7	-                                               \
      1  1                                                              \
                                                                        \
      1  1	14	-                                               \
      1  0                                                              \
                                                                        \
      1  0	9	-                                               \
      0  1                                                              \
   */                                                                   \
  static const int8_t enc_states[] = { 0, 0, 0, 0, 0, 0,                \
				       +1, -1,                          \
				       0,                               \
				       -1,                              \
				       0,                               \
				       +1,                              \
				       0,                               \
				       +1, -1, 0 };                     \
  uint8_t state = FETCH_ENCODER_STATE();                                \
  if ( state && state != prev_state )                                   \
    {                                                                   \
      if ( prev_state )                                                 \
        value += enc_states[((prev_state << 2) | state)];               \
      prev_state = state;
    }

ISR(P0_PCINT_vect) { ISR_BODY }

#if P0_PCINT_vect != P1_PCINT_vect
ISR(P1_PCINT_vect) { ISR_BODY }
#endif


int i2c_data_address;

/** Slave-receiver callback for the `twi` library.  Implements
 * register-write functionality; min. two data bytes are required -- first is
 * the register address, second is the data to write.
 */
void
handle_remote_i2c_write(uint8_t* buf, int len)
{
  i2c_data_address = buf[0];
  uint8_t reply = 1;            /* ACK */
  if ( --len > 0 )
    {
      switch ( i2c_data_address )
      {
      case I2C_REGISTER_VALUE:
        if ( len > sizeof(value) )
          /* too big for the value */
          reply = 0;
        else
          value = *((int16_t*)(&buf[1]));
        break;

      case I2C_REGISTER_VALUE_WITH_RESET:
        /* can't write this one */
      default:
        reply = 0;
        break;
      }
      i2c_data_address = -1;
    }

  twi_reply(reply);
}

void
handle_remote_i2c_read()
{
  uint8_t reply = 1;            /* ACK */

  if ( i2c_read_address < 0 )
    /* No data?!  Reply with NACK. */
    reply = 0;
  else
    {
      switch ( i2c_read_address )
        {
        case I2C_REGISTER_VALUE:
          twi_transmit(&value, 2);
          break;

        case I2C_REGISTER_VALUE_WITH_RESET:
          twi_transmit(&value, 2);
          value = 0;
          break;

        default:
          reply = 0;
          break;
        }
      i2c_read_address = -1;
    }
}

void
setup()
{
  /* PB1:0 => inputs */
  DDRB &= ~(_BV(PINB0) | _BV(PINB1));

  /* PB5 (LED pin) => output */
  DDRB |= _BV(PINB5);

  /* PC3:0 => inputs */
  DDRC &= ~(_BV(PINC0)|_BV(PINC1)|_BV(PINC2)|_BV(PINC3));

  /* PD7:5 => inputs */
  DDRD &= ~(_BV(PIND5)|_BV(PIND6)|_BV(PIND7));

  /* Initialize stored encoder state. */
  prev_state = FETCH_ENCODER_STATE();

  /* Enable pin-change interrupt(s) for the input pins. */
  PCICR |= P0_PCIE | P1_PCIE;

  /* Enable those interrupts for only the configured input pins. */
#if P0_PCIE == P1_PCIE
  P0_PCMSK = P0_PCINT | P1_PCINT;
#else
  P0_PCMSK = P0_PCINT;
  P1_PCMSK = P1_PCINT;
#endif

  /* Initialize the I2C stuff. */
  twi_setAddress(DEFAULT_I2C_SLAVE_ADDRESS);
  twi_attachSlaveTxEvent(handle_remote_i2c_read);
  twi_attachSlaveRxEvent(handle_remote_i2c_write);
  twi_init();
}


void
loop()
{
  /* Nothing to do here, since the action all happens in the interrupt service
     routine. */
}
