
// -----
// ModifiedRotaryEncoder.cpp - Library for using rotary encoders.
// This class is implemented for use with the Arduino environment.
// Copyright (c) by Matthias Hertel, http://www.mathertel.de
// This work is licensed under a BSD style license. See http://www.mathertel.de/License.aspx
// More information on: http://www.mathertel.de/Arduino
// -----
// 18.01.2014 created by Matthias Hertel
// 17.06.2015 minor updates.
// -----

#include "Arduino.h"


#define LATCHSTATE 3
#define MAX 50000
#define MIN 0

class ModifiedRotaryEncoder
{
public:
  // ----- Constructor -----
  ModifiedRotaryEncoder(int pin1, int pin2);
  
  // retrieve the current position
  long  getPosition();

  long  getNonLinearPosition();

  // adjust the current position
  void setPosition(long newPosition);

  long getStep();

  void tickSwitch();

  void setNonLinearPosition(long n);

  void dbl();
  void half();

  // call this function every some milliseconds or by using an interrupt for handling state changes of the rotary encoder.
  void tick(void);

private:
  int _pin1, _pin2; // Arduino pins used for the encoder. 
  
  int8_t _oldState;
  
  long _position;     // Internal position (4 times _positionExt)
  long _positionExt;  // External position
  long nonLinearPosition;
  long  step;
};



// The array holds the values –1 for the entries where a position was decremented,
// a 1 for the entries where the position was incremented
// and 0 in all the other (no change or not valid) cases.

const int8_t KNOBDIR[] = {
  0, -1,  1,  0,
  1,  0,  0, -1,
  -1,  0,  0,  1,
0,  1, -1,  0  };


// positions: [3] 1 0 2 [3] 1 0 2 [3]
// [3] is the positions where my rotary switch detends
// ==> right, count up
// <== left,  count down


// ----- Initialization and Default Values -----

ModifiedRotaryEncoder::ModifiedRotaryEncoder(int pin1, int pin2) {
  
  // Remember Hardware Setup
  _pin1 = pin1;
  _pin2 = pin2;
  
  // Setup the input pins
  pinMode(pin1, INPUT);
  digitalWrite(pin1, HIGH);   // turn on pullup resistor

  pinMode(pin2, INPUT);
  digitalWrite(pin2, HIGH);   // turn on pullup resistor

  // when not started in motion, the current state of the encoder should be 3
  _oldState = 3;

  // start with position 0;
  _position = 0;
  _positionExt = 0;
  nonLinearPosition = MIN;
  step = 100000;
  
} // ModifiedRotaryEncoder()


long  ModifiedRotaryEncoder::getPosition() {
  return _positionExt;
} // getPosition()


long  ModifiedRotaryEncoder::getNonLinearPosition() {
  return nonLinearPosition;
}


void  ModifiedRotaryEncoder::setNonLinearPosition(long n) {
  if ((n > MAX))
    n = MAX;
  else if (n < MIN)
      n = MIN;
    else 
      nonLinearPosition = n;
} 

long  ModifiedRotaryEncoder::getStep() {
  return step;
}

void ModifiedRotaryEncoder::dbl() {
  setNonLinearPosition(2 * nonLinearPosition);
  
}

void ModifiedRotaryEncoder::half() {
  setNonLinearPosition( nonLinearPosition / 2);
}



void  ModifiedRotaryEncoder::tickSwitch() {
  if (step == 1) 
    step = 10000;
  else
    step /= 10;
} // getPosition()


void ModifiedRotaryEncoder::setPosition(long newPosition) {
  // only adjust the external part of the position.
  _position = ((newPosition<<2) | (_position & 0x03L));
  _positionExt = newPosition;
} // setPosition()


void ModifiedRotaryEncoder::tick(void)
{
  int sig1 = digitalRead(_pin1);
  int sig2 = digitalRead(_pin2);
  int8_t thisState = sig1 | (sig2 << 1);


  int old_position = _position;
  if (_oldState != thisState) {
    _position += KNOBDIR[thisState | (_oldState<<2)];


    if (thisState == LATCHSTATE) {
        
      int diff = (_position >> 2) - _positionExt;
      long new_value = nonLinearPosition + diff * step;
       
      if ((new_value > MIN) && (new_value < MAX)) {
          nonLinearPosition = new_value;
         
         
       } ;
       _positionExt = _position >> 2;
     
     
    }
      
    _oldState = thisState;
  } // if
} // tick()

