// Branch: Tri Motor Speed Control w/o PID

// See related posts -
// http://rcarduino.blogspot.co.uk/2012/01/how-to-read-rc-receiver-with.html
// http://rcarduino.blogspot.co.uk/2012/03/need-more-interrupts-to-read-more.html
// http://rcarduino.blogspot.co.uk/2012/01/can-i-control-more-than-x-servos-with.html
// include the pinchangeint library - see the links in the related topics section above for details
//#include <PinChangeInt.h>
#include <PinChangeInterrupt.h>
#include <PinChangeInterruptBoards.h>
#include <PinChangeInterruptPins.h>
#include <PinChangeInterruptSettings.h>
#include <Servo.h>

// Assign your channel in pins (from receiver)
#define THROTTLE_IN_PIN 3
#define STEERING_IN_PIN 2
#define PITCH_IN_PIN 4

// Assign your channel out pins (To motors)
#define TOP_OUT_PIN 9 // Outputs to top motor
#define BOTTOM_OUT_PIN 10 // Outputs to bottom motor
#define PITCH_OUT_PIN 11 //Outputs to enable pin on L293 H-Bridge motor driver
#define DRIVER_IN_A 12 // Right: 0, Left: 1
#define DRIVER_IN_B 13 // Right: 1, Left: 0

// Servo objects generate the signals expected by Electronic Speed Controllers and Servos
// We will use the objects to output the signals we read in
// this example code provides a straight pass through of the signal with no custom processing
Servo servoTOP;
Servo servoBOT;

// These bit flags are set in bUpdateFlagsShared to indicate which
// channels have new signals ---> 8 bit flag, ex. 0000011 = Throttle and Steering flags set
#define THROTTLE_FLAG 1
#define STEERING_FLAG 2
#define PITCH_FLAG 1

// holds the update flags defined above
volatile uint8_t bUpdateFlagsShared;
volatile uint8_t bUpdateFlagsShared_Pitch;

// shared variables are updated by the ISR and read by loop.
// In loop we immediatley take local copies so that the ISR can keep ownership of the
// shared ones. To access these in loop
// we first turn interrupts off with noInterrupts
// we take a copy to use in loop and the turn interrupts back on
// as quickly as possible, this ensures that we are always able to receive new signals
volatile uint16_t unThrottleInShared;
volatile uint16_t unSteeringInShared;
volatile uint16_t unPitchInShared;

// These are used to record the rising edge of a pulse in the calcInput functions
// They do not need to be volatile as they are only used in the ISR. If we wanted
// to refer to these in loop and the ISR then they would need to be declared volatile
uint32_t ulThrottleStart;
uint32_t ulSteeringStart;
uint32_t ulPitchStart; // PICK UP HERE

void setup()
{
  Serial.begin(9600);

  // Assign output pins
  pinMode(PITCH_OUT_PIN, OUTPUT);
  pinMode(DRIVER_IN_A, OUTPUT);
  pinMode(DRIVER_IN_B, OUTPUT);

// Tail motor driver pin initialization
  digitalWrite(DRIVER_IN_A, LOW);
  digitalWrite(DRIVER_IN_B, LOW);
 
  Serial.println("multiChannels");

  // attach servo objects, these will generate the correct
  // pulses for driving Electronic speed controllers, servos or other devices
  // designed to interface directly with RC Receivers 
  servoTOP.attach(TOP_OUT_PIN);
  servoBOT.attach(BOTTOM_OUT_PIN);

  // using the PinChangeInt library, attach the interrupts
  // used to read the channels
  attachInterrupt(digitalPinToInterrupt(STEERING_IN_PIN), calcSteering,CHANGE);
  attachInterrupt(digitalPinToInterrupt(THROTTLE_IN_PIN), calcThrottle,CHANGE);
  attachPCINT(digitalPinToPCINT(PITCH_IN_PIN), calcPitch, CHANGE);
}

void loop()
{
  // create local variables to hold a local copies of the channel inputs
  // these are declared static so that thier values will be retained
  // between calls to loop.
  static uint16_t unThrottleIn;
  static uint16_t unSteeringIn;
  static uint16_t unPitchIn;
  static uint16_t difference;
  static uint16_t unMotorSpeed1; // variable that stores overall motor speed --> Top Motor
  static uint16_t unMotorSpeed2; // variable that stores overall motor speed --> Bottom Motor
  static uint8_t bUpdateFlags; // local copy of update flags
  static uint8_t bUpdateFlag_Pitch;
  static uint8_t unPitchMotorSpeed;

  // check shared update flags to see if any channels have a new signal
  if(bUpdateFlagsShared || bUpdateFlagsShared_Pitch)
  {
    noInterrupts(); // turn interrupts off quickly while we take local copies of the shared variables

    // take a local copy of which channels were updated in case we need to use this in the rest of loop
    bUpdateFlags = bUpdateFlagsShared;
    bUpdateFlag_Pitch = bUpdateFlagsShared_Pitch;
   
    // in the current code, the shared values are always populated
    // so we could copy them without testing the flags
    // however in the future this could change, so lets
    // only copy when the flags tell us we can.
   
    if(bUpdateFlags & THROTTLE_FLAG)
    {
      unThrottleIn = unThrottleInShared;
    }
   
    if(bUpdateFlags & STEERING_FLAG)
    {
      unSteeringIn = unSteeringInShared;
    }

    if(bUpdateFlag_Pitch & PITCH_FLAG)
    {
      unPitchIn = unPitchInShared;
    }
    
    // clear shared copy of updated flags as we have already taken the updates
    // we still have a local copy if we need to use it in bUpdateFlags
    bUpdateFlagsShared = 0;
    bUpdateFlagsShared_Pitch = 0;
   
    interrupts(); // we have local copies of the inputs, so now we can turn interrupts back on
    // as soon as interrupts are back on, we can no longer use the shared copies, the interrupt
    // service routines own these and could update them at any time. During the update, the
    // shared copies may contain junk. Luckily we have our local copies to work with :-)
  }

  //Serial.println(unSteeringIn);
  
  // do any processing from here onwards
  // only use the local values unAuxIn, unThrottleIn and unSteeringIn, the shared
  // variables unAuxInShared, unThrottleInShared, unSteeringInShared are always owned by
  // the interrupt routines and should not be used in loop
 
  // the following code provides simple pass through
  // this is a good initial test, the Arduino will pass through
  // receiver input as if the Arduino is not there.
  // This should be used to confirm the circuit and power
  // before attempting any custom processing in a project.
 
  // we are checking to see if the channel value has changed, this is indicated 
  // by the flags. For the simple pass through we don't really need this check,
  // but for a more complex project where a new signal requires significant processing
  // this allows us to only calculate new values when we have new inputs, rather than
  // on every cycle.

  if(unSteeringIn < 1460) // Turning Left
  {
    difference = 1460 - unSteeringIn;
    if(unThrottleIn - difference >= 0)
    {
      unMotorSpeed1 = unThrottleIn - difference // +/- pid_m1_out;
      if(unThrottleIn + difference < 2000)
      {
        unMotorSpeed2 = unThrottleIn + difference // +/- pid_m1_out with more processing of pid variable here or in pid loop;
      }
    }
  }
  
  else if(unSteeringIn > 1500) // Turning Right
  {
    difference = unSteeringIn - 1500;
    if(unThrottleIn + difference < 2000)
    {
      unMotorSpeed1 = unThrottleIn + difference;
      if(unThrottleIn - difference >= 0)
      {
        unMotorSpeed2 = unThrottleIn - difference;
      }
    }
  }
  
  else
  {
    unMotorSpeed1 = unThrottleIn;
    unMotorSpeed2 = unThrottleIn;  
  }
  

  //Serial.println(unMotorSpeed1);
  //Serial.println(unMotorSpeed2);

  if(bUpdateFlags)
  {
    if(servoTOP.readMicroseconds() != unMotorSpeed1)
    {
      servoTOP.writeMicroseconds(unMotorSpeed1);
    }
    
    if(servoBOT.readMicroseconds() != unMotorSpeed2)
    {
      servoBOT.writeMicroseconds(unMotorSpeed2);
    }
  }

// CW + Speed Control
if(unPitchIn > 1490)
{
  digitalWrite(DRIVER_IN_A, LOW);
  digitalWrite(DRIVER_IN_B, HIGH);
  unPitchMotorSpeed = map(unPitchIn, 1490, 1920, 0, 255);
}

// CCW + Speed Control
else if(unPitchIn < 1460)
{
  digitalWrite(DRIVER_IN_A, HIGH);
  digitalWrite(DRIVER_IN_B, LOW);
  unPitchMotorSpeed = map(unPitchIn, 1470, 1040, 0, 255);
}

else
{
  unPitchMotorSpeed = 0;
}

Serial.println(unPitchMotorSpeed);

if(bUpdateFlag_Pitch)
{
  //if(analogRead(PITCH_IN_PIN) != unPitchIn)
  //{
    analogWrite(PITCH_OUT_PIN, unPitchMotorSpeed);
  //}
}

  // Reset flags
  bUpdateFlags = 0; 
  bUpdateFlag_Pitch = 0;
}


// simple interrupt service routine

void calcThrottle()
{
  // if the pin is high, its a rising edge of the signal pulse, so lets record its value
  if(digitalRead(THROTTLE_IN_PIN) == HIGH)
  {
    ulThrottleStart = micros();
  }
  else
  {
    // else it must be a falling edge, so lets get the time and subtract the time of the rising edge
    // this gives use the time between the rising and falling edges i.e. the pulse duration.
    unThrottleInShared = (uint16_t)(micros() - ulThrottleStart); // pulse duration
    // use set the throttle flag to indicate that a new throttle signal has been received
    bUpdateFlagsShared |= THROTTLE_FLAG;
  }
}

void calcSteering()
{
  if(digitalRead(STEERING_IN_PIN) == HIGH)
  {
    ulSteeringStart = micros();
  }
  else
  {
    unSteeringInShared = (uint16_t)(micros() - ulSteeringStart); // pulse duration
    bUpdateFlagsShared |= STEERING_FLAG;
  }
}

void calcPitch()
{
  if(digitalRead(PITCH_IN_PIN) == HIGH)
  {
    ulPitchStart = micros();
  }
  else
  {
    unPitchInShared = (uint16_t)(micros() - ulPitchStart); // pulse duration
    bUpdateFlagsShared_Pitch |= PITCH_FLAG;
  }
}
