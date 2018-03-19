/**
 * The MySensors Arduino library handles the wireless radio link and protocol
 * between your home built sensors/actuators and HA controller of choice.
 * The sensors forms a self healing radio network with optional repeaters. Each
 * repeater and gateway builds a routing tables in EEPROM which keeps track of the
 * network topology allowing messages to be routed to nodes.
 *
 * Created by Henrik Ekblad <henrik.ekblad@mysensors.org>
 * Copyright (C) 2013-2015 Sensnology AB
 * Full contributor list: https://github.com/mysensors/Arduino/graphs/contributors
 *
 * Documentation: http://www.mysensors.org
 * Support Forum: http://forum.mysensors.org
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 2 as published by the Free Software Foundation.
 *
 *******************************
 *
 * REVISION HISTORY
 * Version 1.0 - Thomas Kocher
 *
 * DESCRIPTION
 * This sketch implements a MySensors 4 channel ampere meter sensor.
 * It uses the OpenEnergyMonitor libraries: https://openenergymonitor.org
 * The communication is RS485 based.
 */

// Enable debug prints to serial monitor
#define MY_DEBUG


/**
 * Configure RS485 transport
 */
// Enable RS485 transport layer
#define MY_RS485

// Incontrast to radio based transport, we have to explicitely assigen a MY_NODE_ID whe using RS485 transport.
#define MY_NODE_ID 46

// Define this to enables DE-pin management on defined pin
#define MY_RS485_DE_PIN 2

// Set RS485 baud rate to use
#define MY_RS485_BAUD_RATE 9600


/**
 * Configure sensor
 */
#define SENSOR_NAME "4Chan Ampere Meter"
#define SENSOR_VERSION "1.0"

// Analog pin assignment for all channels of ampere meter
#define AMP_METER_CH1_PIN A0
#define AMP_METER_CH2_PIN A1
#define AMP_METER_CH3_PIN A2
#define AMP_METER_CH4_PIN A3

// Digital I/O pin number for status led
#define STATUS_PIN 13
#define STATUS_BLINK_INTERVAL 200

// The sketch reads approximately 106 samples of current in each cycle of mains at 50 Hz.
// 1480 samples therefore works out at 14 cycles of mains. That will give you a good average reading.
// You can change the number, but you should get as close as possible to having a whole number of mains cycles,
// otherwise if you have only part of a cycle on the end, you will introduce an error.
#define NUM_OF_SAMPLES 1480

#define NUM_OF_CHANNELS 4


/**
 * Include libraries
 */
#include <MySensors.h>
#include "EmonLib.h"

/**
 * Sensor code
 */
MyMessage msgAmp[NUM_OF_CHANNELS];

EnergyMonitor emon[NUM_OF_CHANNELS];

// measure current on each channel and send values every 5secs
static const uint64_t UPDATE_INTERVAL = 5000;


/**
 * Initialize Ampere meters.
 */
void before() {
  Serial.print("Initializing "); Serial.print(SENSOR_NAME); Serial.print(" v"); Serial.print(SENSOR_VERSION); Serial.println("...");

  pinMode(STATUS_PIN, OUTPUT);
  digitalWrite(STATUS_PIN, LOW);

  for (int i = 0; i < NUM_OF_CHANNELS; i++) {
    msgAmp[i].sensor = i;
    msgAmp[i].type = V_CURRENT;
  }

  // initialize apere meter with pin and calibration current constant
  // see https://learn.openenergymonitor.org/electricity-monitoring/ctac/ct-and-ac-power-adaptor-installation-and-calibration-theory
  // calculation: current constant = (ratio of current transformer) / (burden resistor)
  // (100 / 0.050) / 33 = 60.6
  emon[0].current(AMP_METER_CH1_PIN, 60.6);
  emon[1].current(AMP_METER_CH2_PIN, 61.6);
  emon[2].current(AMP_METER_CH3_PIN, 60.0);
  emon[3].current(AMP_METER_CH4_PIN, 60.6);
}


/**
 * Present the sonsor the contrller.
 */
void presentation () {
  // Send the sketch version information to the gateway
  sendSketchInfo(SENSOR_NAME, SENSOR_VERSION);

  // Register all sensors to gw (they will be created as child devices)
  for (int i = 0; i < NUM_OF_CHANNELS; i++) present(i, S_MULTIMETER);
}


/**
 * Setup
 */
void setup() {
  // everthing was initialized in before(). hence do nothing
}


/**
 * Loop
 */
void loop() {
  // put your main code here, to run repeatedly:

  for (int i = 0; i < NUM_OF_CHANNELS; i++) {
    // Calculate RMS currernt with defined number of samples
    float Irms = (float)emon[i].calcIrms(NUM_OF_SAMPLES);
    send(msgAmp[i].set(Irms, 3));
    blinkStatus();
  }

  sleep(UPDATE_INTERVAL);
}


/**
 * Blink status LED once.
 */
void blinkStatus() {
  digitalWrite(STATUS_PIN, HIGH);
  wait(STATUS_BLINK_INTERVAL);
  digitalWrite(STATUS_PIN, LOW);
}
