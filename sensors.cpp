/***********************************************************************
 * Created by Peter Harrison on 30/12/2017.
 * Copyright (c) 2017 Peter Harrison
 *
 *
 * Permission is hereby granted, free of charge, to any person obtaining a
 * copy of this software and associated documentation files (the
 * "Software"), to deal in the Software without restriction, including
 * without l> imitation the rights to use, copy, modify, merge, publish,
 * distribute, sublicense, and/or sell copies of the Software, and to
 * permit persons to whom the Software is furnished to do so, subject to
 * the following conditions:
 *
 * The above copyright notice and this permission notice shall be included
 * in all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS
 * OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
 * IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY
 * CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT,
 * TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE
 * SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 *
 **************************************************************************/

#include "sensors.h"
#include "parameters.h"
#include "src/hardware/hardware.h"


bool sensorsEnabled;
// sensor values
volatile int sensFR;
volatile int sensFL;
volatile int sensR;
volatile int sensL;

volatile int frontSum;
volatile int frontDiff;

int rawFR;
int rawFL;
int rawR;
int rawL;


volatile int sensorState;


// sensor wall detection
volatile bool wallSensorRight;
volatile bool wallSensorLeft;
volatile bool wallSensorFrontRight;
volatile bool wallSensorFrontLeft;
volatile bool wallSensorFront;

const PROGMEM int frontSumTable[] = {
  485,  485,  484,  483,  481,  479,  477,  475,  473,  470,  467,  465,  462,  460,  456,  454,  450,  447,  445,  440,  435,  433,  425,  421,  410,  398,  393,  379,  367,  361,  352,  345,  334,  325,  319,  311,  303,  299,  291,  287,  280,  275,  271,  264,  260,  255,  250,  247,  243,  236,  234,  231,  226,  224,  222,  217,  214,  210,  208,  206,  202,  200,  199,  196,  193,  192,  190,  186,  185,  184,  181,  179,  178,  176,  175,  173,  172,  171,  170,  169,  166,  166,  163,  163,  163,  161,  160,  160,  158,  157,  157,  154,  153,  153,  152,  151,  150,  149,  149,  147,  147,  147,  146,  144,  145,  143,  143,  141,  141,  141,  140,  140,  138,  139,  138,  138,  136,  137,  135,  135,  134,  134,  133,  133,  133,  131,  133,
};

const PROGMEM int frontDiffTable[] = { -15,  -15,  -14,  -15,  -15,  -15,  -15,  -15,  -15,  -16,  -15,  -15,  -14,  -14,  -14,  -14,  -14,  -13,  -13,  -12,  -11,  -11,   -9,   -7,   -4,    0,   -1,    1,    1,    1,    2,    1,    2,    1,    1,    1,    1,    1,    1,    1,    0,    1,    1,    0,    0,    1,    0,    1,   -1,    0,    0,   -1,    0,    0,    0,   -1,    0,    0,    0,    0,    0,    0,   -1,    0,   -1,    0,    0,    0,   -1,    0,   -1,    1,    0,    0,   -1,    1,    0,   -1,    0,   -1,    0,    0,   -1,   -1,    1,   -1,    0,    0,    0,    1,    1,    0,    1,    1,    0,    1,    0,    1,    1,    1,    1,    1,    0,    2,    1,    1,    1,    1,    1,    1,    2,    0,    2,    1,    2,    2,    2,    1,    3,    1,    2,    0,    1,    3,    1,    3,    1,};

void sensorsInit() {
  sensorState = 0;
}

/***
 * Sensor reading takes 8 states.
 * The initial state of zero is only entered once and ensures all emitters off.
 * Time needed per state is between 15 and 30 us and the update is called at
 * a frequency of 4kHz so the load is between about 5% and 10%. Quite high.
 * It looks like some channels are slower to read than others!
 *
 * Note that the ADC hardware is significantly speeded up in hardwareInit();
 *
 * With this scheme, sensors are read only 500 times per second. That is fine for a
 * slow robot but significant improvements will be needed if speed increases.
 *
 * sensorUpdate takes about 600us to run and it could be made more efficient by
 * use of a state machine or by only calling it when the answers are needed
 *
 * All told, reading the sensors takes about 25% of the available processor time.
 *
 * Also, time wasted here robs the processor of valuable power for maze flooding
 * and control tasks.
 */

void sensorUpdate() {
  // gather all the unlit readings
  int darkL = analogRead(LEFT_DIAG);
  int darkR = analogRead(RIGHT_DIAG);
  int darkFL = analogRead(LEFT_FRONT);
  int darkFR = analogRead(RIGHT_FRONT);
  digitalWrite(LED_TX_RF, 1);	// front LEDs on
  digitalWrite(LED_TX_LF, 1);
  delayMicroseconds(50);
  int litFL = analogRead(LEFT_FRONT);
  int litFR = analogRead(RIGHT_FRONT);
  digitalWrite(LED_TX_RF, 0);	// front LEDs off
  digitalWrite(LED_TX_LF, 0);
  digitalWrite(LED_TX_RD, 1);	// side LEDs on
  digitalWrite(LED_TX_LD, 1);
  delayMicroseconds(50);
  int litL = analogRead(LEFT_DIAG);
  int litR = analogRead(RIGHT_DIAG);
  digitalWrite(LED_TX_RD, 0);	// side LEDs off
  digitalWrite(LED_TX_LD, 0);

  // all the dark and lit values have now been collected so
  // see what we got from reflection but never accept negative readings
  rawFL = max(litFL - darkFL, 0);
  rawFR = max(litFR - darkFR, 0);
  rawL = max(litL - darkL, 0);
  rawR = max(litR - darkR, 0);

  // now calculate the normalised readings
  sensFL = (rawFL * LF_NOMINAL) / LF_CAL;
  sensFR = (rawFR * RF_NOMINAL) / RF_CAL;
  sensL = (rawL * LD_NOMINAL) / LD_CAL;
  sensR = (rawR * RD_NOMINAL) / RD_CAL;

  frontSum = sensFL + sensFR;
  frontDiff = sensFL - sensFR;
  // there is some hysteresis built in to the side sensors to ensure
  // cleaner edges
  // decide whether walls are present - use both sensors at the front

  wallSensorFrontRight = sensFR > FRONT_THRESHOLD;
  wallSensorFrontLeft = sensFL > FRONT_THRESHOLD;
  wallSensorFront = wallSensorFrontRight && wallSensorFrontLeft;

  if (sensL < (DIAG_THRESHOLD)) {
wallSensorLeft = false;
  } else  if (sensL > (DIAG_THRESHOLD + 5)) {
wallSensorLeft = true;
  }
  if (sensR < (DIAG_THRESHOLD)) {
wallSensorRight = false;
  } else if (sensR > (DIAG_THRESHOLD + 5)) {
wallSensorRight = true;
  }

}



void sensorsEnable() {
  sensorsEnabled = true;
}

void sensorsDisable() {
  sensorsEnabled = false;
}

int sensorGetFrontDistance() {
  int i;
  for (i = 0; i < 127; i++) {
if (frontSum >= (int)pgm_read_word_near(frontSumTable + i)) {
  break;
}
  }
  return i;
}

int sensorGetFrontSteering(int distance) {
  int i = 0;
  if (distance >= 0 && distance < 127) {
i = frontDiff - pgm_read_word_near(frontDiffTable + distance);
  }
  return i;
}
