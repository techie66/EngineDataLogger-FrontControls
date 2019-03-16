/*    Basic Idea
 *  loop reads all input pins and checks for serial commands
 *  create command word and sensor data stream
 *  send sensor data (limit rate by time, etc)
 *  call each action function
 *  each function performs small snippet based on command word (smallest action at a time)
 *    use static vars to maintain state (eg blinking = on first time, off second time)
 *    millis() to maintain timing and synchronization
 *  brake light on interrupt
 *    flash initially, steady until low
 *  horn sounds on interrupt
 *  
 *  Pins read full register at a time and bitmasking used heavily to execute commands
 *  
 *  brake()
 *  blinkRight()
 *  blinkLeft()
 *  hornSound()
 *  enableStart()
 *  hlMode()
 *  
 */

#include <avr/interrupt.h>
#include <avr/power.h>
#include <avr/sleep.h>
 
uint8_t inputCmdD; // pins 0-7
uint8_t inputCmdB; // pins 8-13 (two high bits unusable) (all outputs)
uint8_t inputCmdC; // analog pins A0-A5 (two high bits unuseable ATMega328p)
uint8_t serialCmdD = 0;
uint8_t serialCmdB = 0; // Extra Input, used for overrides D
uint8_t serialCmdC = 0;
uint8_t serialCmdA = 0; // Extra Input, used for overrides C
uint8_t receivedCmdD;
uint8_t receivedCmdC;
uint8_t ADCSRA_save = 0;
volatile boolean brakeStart = false;
boolean engineStarted = false;
float systemVoltage = 0;

// PORTD
const uint8_t brakeInPin = 2;
const uint8_t hornInPin = 3;
const uint8_t leftInPin = 4;
const uint8_t rightInPin = 5;
const uint8_t hlhighInPin = 6;
const uint8_t killInPin = 7;

// PORTB
const uint8_t leftOutPin = 8;
const uint8_t rightOutPin = 9;
const uint8_t hornOutPin = 10;
const uint8_t hlhighOutPin = 11;
const uint8_t hllowOutPin = 12;
const uint8_t brakeOutPin = 13;

// PORTC
const uint8_t startEnableOutPin = A0;
const uint8_t rearOutPin = A1;
const uint8_t clutchInPin = A2;
const uint8_t kickInPin = A3;
const uint8_t neutralInPin = A4;
const uint8_t voltageInPin = A5;

const uint8_t CMD_SIZE = 4;  // WIP see doCmd()
const uint16_t SERIAL_OUT_RATE = 50;
const unsigned long SERIAL_EXPIRE = 4000;
const uint8_t RLY_ON = LOW;
const uint8_t RLY_OFF = HIGH;
const unsigned long blinkDelay = 350;
const unsigned long SLEEP_DELAY = 120000;
const unsigned long BRAKE_FLASH_INTERVAL = 50;

//Constants for voltage divider
const int resistor1 = 991; //997 (992 meas)
const int resistor2 = 223; //236 (218 meas)
const float denominator = (float)resistor2 / (resistor1 + resistor2);

// Constants for Cmd flags
const uint8_t ENGINE_RUNNING = B10000000;       // serialCmdA
const uint8_t BRAKE_ON = B00000100;             //receivedCmdD
const uint8_t HORN_ON = B00001000;              //receivedCmdD
const uint8_t LEFT_ON = B00010000;              //receivedCmdD
const uint8_t RIGHT_ON = B00100000;             //receivedCmdD
const uint8_t HIGH_BEAMS_ON = B01000000;        //receivedCmdD
const uint8_t KILL_ON = B10000000;              //receivedCmdD

const uint8_t CLUTCH_DISENGAGED = B00000100;    //receivedCmdC
const uint8_t IN_NEUTRAL  = B00010000;          //receivedCmdC
const uint8_t KICKSTAND_UP = B00001000;         //receivedCmdC
//ADD MORE HERE


void readSensors() {
  inputCmdD = PIND;
  inputCmdC = PINC;
  systemVoltage = ( analogRead(voltageInPin));
  systemVoltage = (systemVoltage / 1024) * 5.0;
  systemVoltage = systemVoltage / denominator;

  // invert ACTIVE LOW signal inputs
  inputCmdC ^= B00011100;

}

void recvCmd() {
  // read CMD_SIZE command from serial(USB)
  // TODO start and stop chars, length?
  static uint8_t receivedByte[(CMD_SIZE+1)];
  static uint8_t numBytesRecv = (CMD_SIZE+1);
  unsigned long currentMillis = millis();
  static unsigned long lastMillis = 0;
  boolean goodRead = false;

  if (currentMillis > lastMillis + SERIAL_EXPIRE) {
    serialCmdD = 0;
    serialCmdB = 0;
    serialCmdC = 0;
    serialCmdA = 0;
  }

  
  while (Serial.available() > 0) {
    if (numBytesRecv < CMD_SIZE) {
      receivedByte[numBytesRecv] = Serial.read();
      //Serial.print("ReadByte ");
      //Serial.println(numBytesRecv);
      numBytesRecv++;
    }
    else if (numBytesRecv == CMD_SIZE) {
      if ((char)Serial.read() == 'Z') {
        // End Character. Presumably good data
        goodRead = true;
        numBytesRecv++;
        //Serial.println("GoodRead");
        break;
      }
      else {
        //Serial.println("Not a Z");
        //numBytesRecv++;
      }
    }
    else if (numBytesRecv > CMD_SIZE) {
      if ((char)Serial.read() == 'A') {
        // Start Character
        //Serial.println("Start");
        numBytesRecv = 0;
      }
    }
  }

  if (goodRead) {
    // Written for CMD_SIZE = 4
    
    serialCmdD = receivedByte[0]; // Equivalent to PIND
    serialCmdB = receivedByte[1]; // Overrides for PIND
    serialCmdC = receivedByte[2]; // Equivalent to PINC
    serialCmdA = receivedByte[3]; // Overrides for PINC
    if (serialCmdA & B10000000) {
      engineStarted = true;
    }
    // zero out local vars
    //numBytesRecv = 0;
    goodRead = false;
    for (uint8_t i = 0; i < (CMD_SIZE+1); i++ ) {
      receivedByte[i] = 0;
    }
    // Set Time of last received CMD
    lastMillis = currentMillis;
  }
  
}

void sendData() {
  static uint8_t lastCmdD;
  static uint8_t lastCmdC;
  static float lastVoltage;
  unsigned long currentMillis = millis();
  static unsigned long lastMillis = 0;
  uint8_t tmp;


  if ((inputCmdD == lastCmdD) && (inputCmdC == lastCmdC) && (systemVoltage == lastVoltage)) {
    //Nothing, everything is the same, no need to repeat ourselves
  }
  else if (currentMillis < (lastMillis + SERIAL_OUT_RATE)) {
    //Nothing, limit output rate
  }
  else {
    // Send sensor/state data over serial(USB)
    Serial.write(inputCmdD);
    Serial.write(inputCmdC);
    Serial.write((byte *) &systemVoltage, 4);
    Serial.println("");
    
    // Hack to output ASCII version
/*    uint8_t copyD = receivedCmdD;
    uint8_t copyC = receivedCmdC;
  
    Serial.println(systemVoltage);
    Serial.print("receivedCmdD:");
    for(int i=0;i<8;i++) {
      if(copyD&B10000000){
        Serial.print("1");
      }
      else {
        Serial.print("0");
      }
      copyD = copyD << 1;
    }
    Serial.println("");
    Serial.print("receivedCmdC:");
    for(int i=0;i<8;i++) {
      if(copyC&B10000000){
        Serial.print("1");
      }
      else {
        Serial.print("0");
      }
      copyC = copyC << 1;
    }
    Serial.println("");
    */
    
  
  lastCmdD = inputCmdD;
  lastCmdC = inputCmdC;
  lastVoltage = systemVoltage;
  lastMillis = currentMillis;
  }
}

ISR(TIMER1_COMPA_vect){
  // Interrupt called when time to toggle flashers
  
  static bool pinOn = false;

  //LEFT
  // Determine if blinking or not, bitmask
  if (receivedCmdD & B00010000) { // Pin 4 of PortD
    //Toggle
    if (pinOn) {
      // Turn off
      digitalWrite(leftOutPin, RLY_OFF);
    }
    else {
      //turn on
      digitalWrite(leftOutPin, RLY_ON);
    }
  }
  else {
    // - Not, turn light on and exit
    digitalWrite(leftOutPin, RLY_OFF);
  }

  //RIGHT
  // Determine if blinking or not, bitmask
  if (receivedCmdD & B00100000) { // Pin 5 of PortD
    //Toggle
    if (pinOn) {
      // Turn off
      digitalWrite(rightOutPin, RLY_OFF);
    }
    else {
      //turn on
      digitalWrite(rightOutPin, RLY_ON);
    }
  }
  else {
    // - Not, turn light on and exit
    digitalWrite(rightOutPin, RLY_OFF);
  }

  if (receivedCmdD & B00110000) { // Either left, right or both, Pin 4 or 5 of PortD
    // Enable rear lights
    digitalWrite(rearOutPin, RLY_ON);
    pinOn = !pinOn;
  }
  else { // Neither
    // Disable rear lights
    digitalWrite(rearOutPin, RLY_OFF);
  }
}

void brakeLight() {
  /*
   * Initial turn on flashes before going steady until brake is released
   */
  static int i = 0;
  static bool brakeEngaged = false;
  static unsigned long brakeDelayStart = millis();
  unsigned long currentMillis = millis();
  
  if (brakeEngaged) {
    brakeStart=false;
  }
  
  if (receivedCmdD & BRAKE_ON) {
    if (brakeStart) {
      /*
       * if--else (i<14) construct prevents from tying up processor
       * while waiting for light to flash
       */
      
      if (i < 14) { // while loop would block for almost a second
        if (currentMillis > ( brakeDelayStart + BRAKE_FLASH_INTERVAL )) {
          digitalWrite(brakeOutPin, !digitalRead(brakeOutPin));
          brakeDelayStart = currentMillis;
          i++;
        }
      }
      else {
        digitalWrite(brakeOutPin,RLY_ON);
        brakeStart = false;
      }
      
    }
    else {
      digitalWrite(brakeOutPin,RLY_ON);
      brakeEngaged = true;
    }
  }
  else {
    digitalWrite(brakeOutPin,RLY_OFF);
    i = 0;
    brakeEngaged = false;
    brakeStart=true;
  }
}

void hornSound() {
  if (receivedCmdD & HORN_ON) {
    digitalWrite(hornOutPin,RLY_ON);
  }
  else {
    digitalWrite(hornOutPin,RLY_OFF);
  }
}

void enableStart() {
  /*
   * Enables / Disables ignition on
   * when disabled, only off or acc
   * when on able to turn on ignition
   * only able to leave ignition on state when enabled
   * 
   * Input: neutral, clutch, kickstand, (engine running), kill switch

   */

   // If either clutch or neutral switches are grounded (engine off or on)
   if (  (receivedCmdC & (IN_NEUTRAL | CLUTCH_DISENGAGED)) && (receivedCmdD & KILL_ON) ) {
     // enable
     digitalWrite(startEnableOutPin, RLY_ON);
   }
   // if kickstand up (engine off)
   else if ( (receivedCmdC & KICKSTAND_UP) && !(serialCmdA & ENGINE_RUNNING) && (receivedCmdD & KILL_ON) ) {
     // enable
     digitalWrite(startEnableOutPin, RLY_ON);
   }
   else if ( engineStarted && !(receivedCmdD & KILL_ON) && (receivedCmdC & (IN_NEUTRAL | CLUTCH_DISENGAGED)) ) {
   //else if (!(receivedCmdD & KILL_ON)) {
     digitalWrite(startEnableOutPin, RLY_ON);
   }
   // Otherwise
   else {
     // disable
     digitalWrite(startEnableOutPin, RLY_OFF);
   }
     
   
}

void hlMode() {
  /*
   * Control two relays for headlights
   * one relay is high, other is low beams
   */
   if ( (receivedCmdD & KILL_ON) && (receivedCmdD & HIGH_BEAMS_ON) && (serialCmdA & ENGINE_RUNNING) ) {
     // high beam switch on, kill switch on, engine running
     digitalWrite(hlhighOutPin, RLY_ON);
     digitalWrite(hllowOutPin, RLY_OFF);
   }
   else if (serialCmdA & ENGINE_RUNNING) {
     digitalWrite(hllowOutPin, RLY_ON);
     digitalWrite(hlhighOutPin, RLY_OFF);
   }
   else {
     digitalWrite(hlhighOutPin, RLY_OFF);
     digitalWrite(hllowOutPin, RLY_OFF);
   }
}

void Wakeup_Routine()
{
  sleep_disable();
  detachInterrupt(0);
  power_all_enable ();                                  // power everything back on
  ADCSRA = ADCSRA_save;
}

void allRelaysOff () {
  //Turns all relays off
  
  digitalWrite(leftOutPin, RLY_OFF);
  digitalWrite(rightOutPin, RLY_OFF);
  digitalWrite(hornOutPin, RLY_OFF);
  digitalWrite(hlhighOutPin, RLY_OFF);
  digitalWrite(hllowOutPin, RLY_OFF);
  digitalWrite(brakeOutPin, RLY_OFF);
  digitalWrite(startEnableOutPin, RLY_OFF);
  digitalWrite(rearOutPin, RLY_OFF);

}

void sleepNow ()
{
  cli();                                                //disable interrupts
  sleep_enable ();                                      // enables the sleep bit in the mcucr register
  attachInterrupt (0, Wakeup_Routine, RISING);          // wake up on RISING level on D2
  allRelaysOff();
  set_sleep_mode (SLEEP_MODE_PWR_DOWN);  
  ADCSRA_save = ADCSRA;
  ADCSRA = 0;                                           //disable the ADC
  sleep_bod_disable();                                  //save power                                              
  sei();                                                //enable interrupts
  sleep_cpu ();                                         // here the device is put to sleep
}

void doCmd() {
  // execute the command
  //TODO figure out priority of serial vs GPIO commands
  /*  On if either On
   *   Use extra cmd space to override off
   *   two bytes to OR with input
   *   two bytes to XOR mask resulting command set
   *   extra bits are status flags (eg. serialCmdA B11000000 and serialCmdB B00000011)
   */
  // Hack to pretend that engine is running
  //serialCmdA |= B10000000;
  
  receivedCmdD = inputCmdD | serialCmdD;
  //receivedCmdB = inputCmdB | serialCmdB; // overrides, Don't need
  receivedCmdC = inputCmdC | serialCmdC;

  /* Enable Overrides
   *  receivedCmdD ^= serialCmdB;  // not complete
   *  receivedCmdC ^= serialCmdA;  // not complete
   */
   
   hornSound();
   hlMode();
   enableStart();
   brakeLight();
  
}

void setup() {
  Serial.begin(115200);

  pinMode(leftOutPin, OUTPUT);
  pinMode(rightOutPin, OUTPUT);
  pinMode(hornOutPin, OUTPUT);
  pinMode(hlhighOutPin, OUTPUT);
  pinMode(hllowOutPin, OUTPUT);
  pinMode(brakeOutPin, OUTPUT);
  pinMode(startEnableOutPin, OUTPUT);
  pinMode(rearOutPin, OUTPUT);

  allRelaysOff();

  pinMode(brakeInPin, INPUT);
  pinMode(hornInPin, INPUT);
  pinMode(leftInPin, INPUT);
  pinMode(rightInPin, INPUT);
  pinMode(hlhighInPin, INPUT);
  pinMode(killInPin, INPUT);
  pinMode(clutchInPin, INPUT_PULLUP);
  pinMode(neutralInPin, INPUT_PULLUP);
  pinMode(kickInPin, INPUT_PULLUP);
  pinMode(voltageInPin, INPUT); // Analog for Voltage
  
  //set timer1 interrupt at 3Hz
  TCCR1A = 0;// set entire TCCR1A register to 0
  TCCR1B = 0;// same for TCCR1B
  TCNT1  = 0;//initialize counter value to 0
  // set compare match register for 3hz increments
  OCR1A = 5208;// = (16*10^6) / (3*1024) - 1 (must be <65536)
  // turn on CTC mode
  TCCR1B |= (1 << WGM12);
  // Set CS10 and CS12 bits for 1024 prescaler
  TCCR1B |= (1 << CS12) | (1 << CS10);  
  // enable timer compare interrupt
  TIMSK1 |= (1 << OCIE1A);

  //Serial.println("Startup Complete!");
}

void loop() {
  static unsigned long sleepWaitStart = 0;
  static boolean sleepCountdown = false;
  readSensors();
  recvCmd();
  doCmd();
  sendData();

  if (systemVoltage < 7) {
    engineStarted = false;
    if (sleepCountdown) {
      if (millis() > sleepWaitStart + SLEEP_DELAY) {
        sleepCountdown = false;
        sleepNow();
      }
    }
    else {
      sleepWaitStart = millis();
      sleepCountdown = true;
    }
  }
  else {
    sleepCountdown = false;
  }
}

