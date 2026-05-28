// Wokwi Custom Chip - For docs and examples see:
// https://docs.wokwi.com/chips-api/getting-started
//
// SPDX-License-Identifier: GPL
// Copyright 2023 Pat Deegan, https://psychogenic.com

#include "wokwi-api.h"
#include <stdio.h>
#include <stdlib.h>

#define I2C_BASE_ADDRESS 0x20
#define NUM_GPIO  8
#define NUM_ADDR_BITS  3

#define CHIPSTATE_FROM(usr_dat) chip_state_t * chip = (chip_state_t*)usr_dat
#define PRINTF_INPUTVALUE(msg, val)   printf("%s 0x%x (p0 0x%x, p1 0x%x)", msg, val, (val & 0xff), ((val & 0xff00) >> 8))

/* Chip state structure 
   Everything we care about and need access to in callbacks.
*/
typedef struct {
  // address bit pins and device address
  uint8_t address;
  pin_t addressBits[NUM_ADDR_BITS];

  // interrupt and "bidir" i/o pins
  pin_t nINT;
  pin_t io[NUM_GPIO];

  // input configuration and read value
  uint16_t inputMask;
  uint16_t inputValue;
  uint16_t lastReadValue;

  // i2c configuration, device and 
  // byte count/port count tracking
  i2c_dev_t i2c_dev;
  i2c_config_t i2c_config;
  uint8_t i2c_portcount;

  // our pin watch config for bidir 
  // io, so we can start/stop watching
  // depending on user settings writes
  pin_watch_config_t io_watch_config;

} chip_state_t;

/* Interrupt flag control 
  Note: inverted logic, i.e. when interrupt is asserted
  the open-drain output is a "switch" tied to ground.
  Otherwise, it is floating.
*/
void interruptFlagOff(chip_state_t* chip) {
  pin_mode(chip->nINT, INPUT);
}

void interruptFlagOn(chip_state_t* chip) {
  pin_mode(chip->nINT, OUTPUT_LOW);
  printf("INT flag SET\n");
}

/*
  All transactions are to be two-bytes long.
  Anything beyond this, in multiples of two, 
  simply restarts/overwrites, like a circular 
  buffer.
*/
void incrementPortCount(chip_state_t* chip) {
  if (chip->i2c_portcount) {
    // already 1, loop back to zero
    chip->i2c_portcount = 0;
  } else {
    chip->i2c_portcount = 1;
  }
}

/*
  I2C connection callback.
  Will tell up the device address and whether this is a read or write 
  operation
*/
bool on_i2c_connect(void *user_data, uint32_t address, bool read) {
  // `address` parameter contains the 7-bit address that was received on the I2C bus.
  // `read` indicates whether this is a read request (true) or write request (false).
  CHIPSTATE_FROM(user_data);

  if (address != chip->address) {
    printf("Getting connects for address 0x%x but am at 0x%x\n", address, chip->address);
    // return true;
  }

  // a new connection
  chip->i2c_portcount = 0;

  // printf("i2c connect for ");
  if (read) {
    printf("Read: reset INT flag\n");
    chip->lastReadValue = chip->inputValue;
    interruptFlagOff(chip);
  } 
  return true; // true means ACK, false NACK
}

/*
  Reading a port as a single byte.
*/
uint8_t on_i2c_read(void *user_data) {
  // printf("i2c Read\n");

  CHIPSTATE_FROM(user_data);
  uint8_t retVal = (chip->inputValue >> (chip->i2c_portcount*8)) & 0xff;
  
  incrementPortCount(chip);

  return retVal; // The byte to be returned to the microcontroller
}

/*
  Writing to a port configures the pins as either
   * output LOW
   * output HIGH / Input (with pull-up)
*/
bool on_i2c_write(void *user_data, uint8_t data) {
  // `data` is the byte received from the microcontroller
  CHIPSTATE_FROM(user_data);

  if (! chip->i2c_portcount) {
    // we are getting bytes, 
    // and this is the first of the 
    // two (or multiples of 2) ports.
    // so reset our input tracking mask
    chip->inputMask = 0;
  }

  for (uint8_t i=0; i<8; i++) {
    // we want our 16-bit index, calculated from 
    // both the bit of this particular byte and accounting
    // for if this is the first or second of the bytes written
    uint8_t bitIdx = i + (chip->i2c_portcount * 8);

    // shortcut to the i/o pin we're processing
    pin_t targetPin = chip->io[bitIdx];

    // disable any watch setup for this pin
    pin_watch_stop(targetPin);

    if (data & (1<<i)) {
      // this is a HIGH/read
      // note it in inputMask, setup up as input with pull-up and watch
      chip->inputMask |= (1 << bitIdx);
      pin_mode(targetPin, INPUT_PULLUP);
      pin_watch(targetPin, &(chip->io_watch_config));
    } else {
      
      // configured as output, tied to GND (low)
      pin_mode(targetPin, OUTPUT_LOW);
    }
  }

  if (chip->i2c_portcount) {
    // we just got the last of a set-of-two bytes
    printf("Input mask is now 0x%x\n", chip->inputMask);
  }

  // get ready for next byte written by incrementing port counter
  incrementPortCount(chip);

  return true; // true means ACK, false NACK
}

void on_i2c_disconnect(void *user_data) {

  // This method is optional. Useful if you need to know when the I2C transaction has concluded.
}

/*  I2C address management
  This is only available on start-up/init, so could in theory 
  simply be handled there.
*/
uint8_t read_address(chip_state_t* chip) {
  uint8_t configuredAddress = I2C_BASE_ADDRESS;
  uint8_t lsbits = 0;
  
  for (uint8_t i=0; i<NUM_ADDR_BITS; i++) {
    if (pin_read(chip->addressBits[i])) {
      lsbits |= (1 << i);
    }
  }

  configuredAddress = configuredAddress | lsbits;

  printf("Chip LSB bits set to %i.  Address is 0x%02x\n", lsbits, configuredAddress);

  return configuredAddress;
}

void chip_addr_change(void *user_data, pin_t pin, uint32_t value) {
  CHIPSTATE_FROM(user_data);
  chip->address = read_address(chip);
}

/* 
  Read the value of the combined input pins, skipping over 
  any that have been configured as output (LOW)
*/
uint16_t readInputsValue(chip_state_t * chip) {
  uint16_t inputsValue = 0;
  for (uint8_t i=0; i<NUM_GPIO; i++) {
    if (chip->inputMask & (1 << i)) {
      // this is an input, we must read
      
      if (pin_read(chip->io[i])) {
          inputsValue |= (1<<i);
      }
    }
  }
  return inputsValue;
}

/*
  Anything that _may_ be treated as an output is watched and,
  on changes, this callback is triggered.
  If the value read in is different than that provided to user
  on last read, we will set the interrupt flag.
  If it is the same, we _clear_ the interrupt flag--this means
  that some changes may be missed by user... yap, but that's 
  how the chip works.
*/
void chip_input_io_change(void *user_data, pin_t pin, uint32_t value) {

  CHIPSTATE_FROM(user_data);
  
  chip->inputValue = readInputsValue(chip);

  if (chip->inputValue != chip->lastReadValue) {
    interruptFlagOn(chip);
  } else {
    interruptFlagOff(chip);
  }

  printf("I/O input changed:");
  PRINTF_INPUTVALUE("from", chip->lastReadValue);
  PRINTF_INPUTVALUE("to", chip->inputValue);
  printf("\n");
  
}

/*
  The chip_state_t structure is initialized here.
  Could do this in chip_init, but there are lots of values
  to set so this is separated out just for clarity.
*/
void initialize_state(chip_state_t * chip) {

  chip->address = I2C_BASE_ADDRESS;
  chip->inputMask = 0xffff;
  chip->inputValue = 0xffff;
  
  chip->i2c_portcount = 0;

  chip->io_watch_config.edge = BOTH;
  chip->io_watch_config.pin_change = chip_input_io_change;
  chip->io_watch_config.user_data = chip;


  i2c_config_t * i2c = &(chip->i2c_config);
  i2c->scl = pin_init("SCL", INPUT_PULLUP);
  i2c->sda = pin_init("SDA", INPUT_PULLUP);
  
  i2c->connect = on_i2c_connect;
  i2c->read = on_i2c_read;
  i2c->write = on_i2c_write;
  i2c->disconnect = on_i2c_disconnect;

  i2c->user_data = chip;
}

/*
  Chip initialization, called on startup.
*/
void chip_init() {
  chip_state_t *chip = malloc(sizeof(chip_state_t));
  const char * addrPinNames[] = {"A0", "A1", "A2"};
  const char * ioPinNames[] = {"P0", "P1", "P2", "P3", "P4", "P5", "P6", "P7"};

  // basic state setup 
  initialize_state(chip);

  chip->nINT = pin_init("nINT", INPUT);

  // address pins monitoring
  const pin_watch_config_t watch_addr_config = {
    .edge = BOTH,
    .pin_change = chip_addr_change,
    .user_data = chip,
  };

  for (uint8_t i=0; i<NUM_ADDR_BITS; i++) {
    chip->addressBits[i] = pin_init(addrPinNames[i], INPUT); 

    pin_watch(chip->addressBits[i], &watch_addr_config);
  }

  for (uint8_t i=0; i<NUM_GPIO; i++) {
    chip->io[i] = pin_init(ioPinNames[i], INPUT_PULLUP); // on power up, high/input
    pin_watch(chip->io[i], &(chip->io_watch_config));
  }
  chip->inputMask = 0xffff;

  chip->address = read_address(chip);
  chip->i2c_config.address = chip->address;
  chip->i2c_dev =  i2c_init(&(chip->i2c_config));

  printf("I2C initialized @ address 0x%x\n", chip->address);

  interruptFlagOff(chip);
}
