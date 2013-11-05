/* http://www.kwartzlab.ca/2010/09/arduino-multi-threading-librar/ */
#include "mthread.h"

// Pin layout
#define APHASE 4
#define AENABLE 5
#define BPHASE 7
#define BENABLE 6

#define AENCODER1 2
#define AENCODER2 8
#define AINTERRUPT 1  // UNO: 0, MICRO: 1
#define BENCODER1 3
#define BENCODER2 9
#define BINTERRUPT 0 // UNO: 1, MICRO: 0

#define TRIG 10
#define ECHO 11
#define BUTTON 12
#define LED 13

class DistanceSensor;
class Led;
class Encoder;
class Motors;
class Button;

DistanceSensor *dist;
Led *led;
Encoder *encA, *encB;
Motors *mots;
Button *button;

//////////////////////////////////////////////////////////////////////////////////

class Motors : public Thread {
  public:
    Motors(int phaseA, int enableA, int phaseB, int enableB);
    void move(int speed, int time);
    void stop();
    void forward(int speed, int time);
    void backward(int speed, int time);
    void left(int speed, int time);
    void right(int speed, int time);
  protected:
    bool loop();
    int phaseA, enableA, phaseB, enableB;
    int seq;
    int speed, time;
};

Motors::Motors(int phaseA, int enableA, int phaseB, int enableB) {
  this->phaseA = phaseA;
  this->enableA = enableA;
  this->phaseB = phaseB;
  this->enableB = enableB;
  pinMode(phaseA, OUTPUT);
  digitalWrite(phaseA, HIGH);
  pinMode(phaseB, OUTPUT);
  digitalWrite(phaseB, HIGH);
  this->seq = 0;
  this->speed = 0;
  this->time = 0;
}

void Motors::move(int speed, int time) {
  analogWrite(enableA, speed);
  analogWrite(enableB, speed); 
  this->speed = speed;
  this->time = time;
  sleep_milli(time);
}

void Motors::stop() {
  analogWrite(enableA, 0);
  analogWrite(enableB, 0); 
  this->speed = 0;
  pause();
}

void Motors::forward(int speed, int time) {
  digitalWrite(phaseA, HIGH);
  digitalWrite(phaseB, HIGH);
  move(speed, time);
}

void Motors::backward(int speed, int time) {
  digitalWrite(phaseA, LOW);
  digitalWrite(phaseB, LOW);
  move(speed, time);
}

void Motors::left(int speed, int time) {
  digitalWrite(phaseA, HIGH);
  digitalWrite(phaseB, LOW);
  move(speed, time);
}

void Motors::right(int speed, int time) {
  digitalWrite(phaseA, LOW);
  digitalWrite(phaseB, HIGH);
  move(speed, time);
}

bool Motors::loop() {
  switch(seq) {
    case 0:
      forward(255, 2000);
      seq++;
      break;
    case 1:
      right(255, 1000);
      seq++;
      break;
    case 2:
      forward(255, 2000);
      seq++;
      break;
    case 3:
      left(255, 1000);
      seq++;
      break;
    case 4:
      backward(255, 2000);
      seq++;
      break;
    case 5:
      left(255, 1000);
      seq++;
      break;
    case 6:
      forward(255, 2000);
      seq++;
      break;
    case 7:
      right(255, 1000);
      seq++;
      break;
    default:
      seq = 0;
  }
  return true;
}

//////////////////////////////////////////////////////////////////////////////////

class Button : public Thread {
  public:
    Button(int pin);
    int isOn;
  protected:
    bool loop();
    int pin;
};

Button::Button(int pin) {
  this->pin = pin;
  this->isOn = 0;
  pinMode(pin, INPUT);
  digitalWrite(pin, LOW);
}

bool Button::loop() {
  int but = digitalRead(pin);
  if(isOn != but) {
    isOn = but;
    // Business logic
    mots->resume();
  }
  sleep_milli(200);
  return true;
}

//////////////////////////////////////////////////////////////////////////////////

class Led : public Thread {
  public:
    Led(int pin);
    void blink(int blinks);
    void switchOn(bool isOn);
    bool isOn;
    int blinks;
  protected:
    bool loop();
    int pin;
};

Led::Led(int pin) {
  this->pin = pin;
  this->blinks = 0;
  this->isOn = false;
  pinMode(pin, OUTPUT);
  digitalWrite(pin, LOW);
  pause();
}

void Led::switchOn(bool isOn) {
  digitalWrite(pin, isOn ? HIGH : LOW);
  this->isOn = isOn;
}

void Led::blink(int blinks) {
  this->blinks = blinks;
  resume();
}

bool Led::loop() {
  if(blinks > 0) {
    switchOn(!isOn);
    if(!isOn) {
      blinks--;
    }
    sleep_milli(200);
  } else {
    pause();
  }
  return true;
}

//////////////////////////////////////////////////////////////////////////////////

class DistanceSensor : public Thread {
  public:
    DistanceSensor(int trig, int echo);
    long d;
  protected:
    int trig, echo;
    bool waiting;
    bool loop();
};

DistanceSensor::DistanceSensor(int trig, int echo) {
  this->trig = trig;
  this->echo = echo;
  this->waiting = false;
  pinMode(trig, OUTPUT);
  digitalWrite(trig, LOW);
  pinMode(echo, INPUT);
}

bool DistanceSensor::loop() {
  if(!waiting) {
    digitalWrite(trig, HIGH);
    waiting = true;
    sleep_micro(10);
  } else {
    digitalWrite(trig, LOW);
    d = pulseIn(echo, HIGH) / 58.2;
    // Business logic
    if(d < 10) {
      mots->stop();
      led->blink(5);
    }
    waiting = false;
    sleep_milli(100);
  }
  return true;
}

//////////////////////////////////////////////////////////////////////////////////

typedef void (*callback)();

class Encoder : public Thread {
  public:
    Encoder(int pin1, int pin2, int interrupt, callback func);
    void onChange();
    int pos;
  protected:
    bool loop();
    int pin1, pin2, interrupt;
};

Encoder::Encoder(int pin1, int pin2, int interrupt, callback func) {
  this->pin1 = pin1;
  this->pin2 = pin2;
  this->interrupt = interrupt;
  this->pos = 0;
  pinMode(pin1, INPUT);
  digitalWrite(pin1, HIGH);
  pinMode(pin2, INPUT);
  digitalWrite(pin2, HIGH);
  attachInterrupt(interrupt, func, CHANGE);
}

void Encoder::onChange() {
  if (digitalRead(pin1) == digitalRead(pin2)) {
    pos--;
  } else {
    pos++;
  }
}

bool Encoder::loop() {
  pause(); // everything is done with the interrupts
  return true;
}

//////////////////////////////////////////////////////////////////////////////////

class SerialMonitor : public Thread {
  public:
    SerialMonitor();
  protected:
    bool loop();
};

SerialMonitor::SerialMonitor() {
  Serial.begin(9600);
  Serial.println("Setup...");
}

bool SerialMonitor::loop() {
  char buf[64];
  sprintf(buf, "A %.5d B %.5d But %i D %u", encA->pos, encB->pos, button->isOn, dist->d);
  Serial.println(buf);
  sleep(1);
  return true;
}

//////////////////////////////////////////////////////////////////////////////////

void doEncoderA() {
  encA->onChange();
}

void doEncoderB() {
  encB->onChange();
}

void setup()  {
  dist = new DistanceSensor(TRIG, ECHO);
  main_thread_list->add_thread(dist);
  encA = new Encoder(AENCODER1, AENCODER2, AINTERRUPT, doEncoderA);
  main_thread_list->add_thread(encA);
  encB = new Encoder(BENCODER1, BENCODER2, BINTERRUPT, doEncoderB);
  main_thread_list->add_thread(encB);
  led = new Led(LED);
  main_thread_list->add_thread(led);
  button = new Button(BUTTON);
  main_thread_list->add_thread(button);
  mots = new Motors(APHASE, AENABLE, BPHASE, BENABLE);
  main_thread_list->add_thread(mots);
  main_thread_list->add_thread(new SerialMonitor());
}

