// PID Library
#include <PID_v1.h>
#include <PID_AutoTune_v0.h>

// Libraries for the DS18B20 Temperature Sensor
#include <OneWire.h>
#include <DallasTemperature.h>

// So we can save and retrieve settings
#include <EEPROM.h>

// Output Relay
#define RelayPin 2

// ************************************************
// Sensor Variables and constants
// Data wire is plugged into port 12 on the Arduino

// One-Wire Temperature Sensor
// (Use GPIO pins for power/ground to simplify the wiring)
#define ONE_WIRE_BUS 12

// Setup a oneWire instance to communicate with any OneWire devices (not just Maxim/Dallas temperature ICs)
OneWire oneWire(ONE_WIRE_BUS);

// Pass our oneWire reference to Dallas Temperature. 
DallasTemperature sensors(&oneWire);

// arrays to hold device address
DeviceAddress tempSensor;

// ************************************************
// PID Variables and constants
// ************************************************

boolean alreadyTuned = false;

//Define Variables we'll be connecting to
double Setpoint;
double Input;
double Output;

volatile long onTime = 0;

// pid tuning parameters
double Kp;
double Ki;
double Kd;

// EEPROM addresses for persisted data
const int SpAddress = 0;
const int KpAddress = 8;
const int KiAddress = 16;
const int KdAddress = 24;

//Specify the links and initial tuning parameters
PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);

// 10 second Time Proportional Output window
int WindowSize = 10000; 
unsigned long windowStartTime;

// ************************************************
// Auto Tune Variables and constants
// ************************************************
byte ATuneModeRemember=2;

double aTuneStep=500;
double aTuneNoise=1;
unsigned int aTuneLookBack=20;

boolean tuning = false;

PID_ATune aTune(&Input, &Output);

// ************************************************
// States for state machine
// ************************************************
enum operatingState { OFF = 0, SETP, RUN, TUNE_P, TUNE_I, TUNE_D, AUTO};
operatingState opState = OFF;

void setup(void) {
  Setpoint = 40;
  Serial.begin(9600);
  Serial.println("starting program");
  
   // Start up the DS18B20 One Wire Temperature Sensor
  
   sensors.begin();
   if (!sensors.getAddress(tempSensor, 0)) 
   {
      Serial.println("no temp sensors!");
   }
   sensors.setResolution(tempSensor, 12);
   sensors.setWaitForConversion(false);
    
   pinMode(RelayPin, OUTPUT);    // Output mode to drive relay
   digitalWrite(RelayPin, LOW);  // make sure it is off to start
  
   // Initialize the PID and related variables
   LoadParameters();
   myPID.SetTunings(Kp,Ki,Kd);
   Serial.print("Kp: ");
   Serial.println(Kp);
   Serial.print("Ki: ");
   Serial.println(Ki);
   Serial.print("Kd: ");
   Serial.println(Kd);
  
   myPID.SetSampleTime(1000);
   myPID.SetOutputLimits(0, WindowSize);
  
  // Run timer2 interrupt every 15 ms so the pulse timing will be accurate
  // even if the arduino is doing something else
  TCCR2A = 0;
  TCCR2B = 1<<CS22 | 1<<CS21 | 1<<CS20;
  
  //Timer2 Overflow Interrupt Enable
  TIMSK2 |= 1<<TOIE2;
  
  
  sensors.requestTemperatures(); // Start an asynchronous temperature reading
  
  //turn the PID on
   myPID.SetMode(AUTOMATIC);
   windowStartTime = millis();
   opState = RUN;
}

// ************************************************
// Timer Interrupt Handler
// ************************************************
SIGNAL(TIMER2_OVF_vect) 
{
  if (opState == OFF)
  {
    digitalWrite(RelayPin, LOW);  // make sure relay is off
  }
  else
  {
    DriveOutput();
  }
}

void loop(void) {
  
  Serial.println("temp: " + String(Input) + " C");
  if (!alreadyTuned && !tuning && false)
  {
    StartAutoTune();
  }
  DoControl();
  delay(1000);
}

// ************************************************
// Called by ISR every 15ms to drive the output
// ************************************************
void DriveOutput()
{
  // Set the output
  // "on time" is proportional to the PID output
  if(millis() - windowStartTime>WindowSize)
  { //time to shift the Relay Window
     windowStartTime += WindowSize;
  }
  if((Output > (millis() - windowStartTime)))
  {
     digitalWrite(RelayPin,HIGH);
  }
  else
  {
     digitalWrite(RelayPin,LOW);
  }
}

// ************************************************
// Execute the control loop
// ************************************************
void DoControl()
{
  // Read the input:
  if (sensors.isConversionAvailable(0))
  {
    Input = sensors.getTempC(tempSensor);
    sensors.requestTemperatures(); // prime the pump for the next one - but don't wait
  }
  
  if (tuning) // run the auto-tuner
  {
     if (aTune.Runtime()) // returns 'true' when done
     {
        FinishAutoTune();
     }
  }
  else // Execute control algorithm
  {
    Serial.println("computing PID");
     myPID.Compute();
  }
  
  // Time Proportional relay state is updated regularly via timer interrupt.
  Serial.println("output time on setpoint: " + String(Output));
  Serial.print("now - windowStartTime: ");
  Serial.println(millis() - windowStartTime);
  Serial.print("setpoint: ");
  Serial.println(Setpoint);
}

void StartAutoTune()
{
  Serial.println("starting autotune");
   // Remember the mode we were in
   ATuneModeRemember = myPID.GetMode();

   // set up the auto-tune parameters
   aTune.SetNoiseBand(aTuneNoise);
   aTune.SetOutputStep(aTuneStep);
   aTune.SetLookbackSec((int)aTuneLookBack);
   tuning = true;
}

void FinishAutoTune()
{
   tuning = false;

   // Extract the auto-tune calculated parameters
   Kp = aTune.GetKp();
   Ki = aTune.GetKi();
   Kd = aTune.GetKd();

   // Re-tune the PID and revert to normal control mode
   myPID.SetTunings(Kp,Ki,Kd);
   myPID.SetMode(ATuneModeRemember);
   
   // Persist any changed parameters to EEPROM
   SaveParameters();
   alreadyTuned = true;
   
   Serial.println("finished autotuning");
}

// ************************************************
// Saves PID parameters Kp, Ki, Kd to EEPROM
// ************************************************
void SaveParameters()
{
   if (Setpoint != EEPROM_readDouble(SpAddress))
   {
      EEPROM_writeDouble(SpAddress, Setpoint);
   }
   if (Kp != EEPROM_readDouble(KpAddress))
   {
      EEPROM_writeDouble(KpAddress, Kp);
   }
   if (Ki != EEPROM_readDouble(KiAddress))
   {
      EEPROM_writeDouble(KiAddress, Ki);
   }
   if (Kd != EEPROM_readDouble(KdAddress))
   {
      EEPROM_writeDouble(KdAddress, Kd);
   }
}

// ************************************************
// Loads PID parameters Kp, Ki, Kd from EEPROM
// ************************************************
void LoadParameters()
{
  // Load from EEPROM
   alreadyTuned = false;
   Setpoint = EEPROM_readDouble(SpAddress);
   Kp = EEPROM_readDouble(KpAddress);
   Ki = EEPROM_readDouble(KiAddress);
   Kd = EEPROM_readDouble(KdAddress);
   
   // Use defaults if EEPROM values are invalid
   if (isnan(Setpoint))
   {
     Setpoint = 40;
   }
   if (isnan(Kp))
   {
     Kp = 50;
     alreadyTuned = false;
   }
   if (isnan(Ki))
   {
     Ki = 0.5;
     alreadyTuned = false;
   }
   if (isnan(Kd))
   {
     Kd = 0.1;
     alreadyTuned = false;
   }  
}

// ************************************************
// Write floating point values to EEPROM
// ************************************************
void EEPROM_writeDouble(int address, double value)
{
   byte* p = (byte*)(void*)&value;
   for (int i = 0; i < sizeof(value); i++)
   {
      EEPROM.write(address++, *p++);
   }
}

// ************************************************
// Read floating point values from EEPROM
// ************************************************
double EEPROM_readDouble(int address)
{
   double value = 0.0;
   byte* p = (byte*)(void*)&value;
   for (int i = 0; i < sizeof(value); i++)
   {
      *p++ = EEPROM.read(address++);
   }
   return value;
}
