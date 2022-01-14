/*
Chance Bisquera, cbisquera21@vt.edu
TMVS Lab
6/24/21 [Updated 10/15/21]
Ankle Exoskeleton Thesis Project

The following script is intended to allow for an operator to manually drive the ankle exoskeleton's R80 T-Motor based on inputs from an array of 3 pushbuttons.
The Arudino is programmed to send commands to the ODrive BLDC motor controller controlling the R80 T-Motor and records data to an SD card.

The following code is written such that the motor operates under ODrive's position control mode.

Segments of the code shown below were informed in part by the "ODriveArduinoTest.ino" file included in the gitHub folder provided by O-Drive.

Required Hardware for Motor Script:
-3 push buttons; Arudino Mega; ODrive BLDC Motor Control; R80 T-Motor

Intended Push Button Layout:
Left PB   -> CCW Rotation, Ball Screw Ascend
Middle PB -> Toggle Motor IDLE/ARMED
Right PB  -> CW Rotation, Ball Screw Descend

Notes:
- For simplicity and safety, this program assumes that the motor is in IDLE mode at the time the program is loaded and started.
- The following script assumes the motor is plugged into Axis1 (M1) of the ODrive
- The following script assumes that the motor, encoder, and ODrive have all been previously calibrated.

*/

#include <HardwareSerial.h>
#include <SoftwareSerial.h>
#include <ODriveArduino.h>

#include<SD.h>
#include<SPI.h>

File dataFile; 

// Printing with stream operator helper functions
template<class T> inline Print& operator <<(Print &obj,     T arg) { obj.print(arg);    return obj; }
template<>        inline Print& operator <<(Print &obj, float arg) { obj.print(arg, 4); return obj; }

////////////////////////////////
// Set up serial pins to the ODrive
// *Don't forget to also connect ODrive GND to Arduino GND. (if the ODrive and Arduino run on two different power sources would it be non-ideal to connect GND)?
////////////////////////////////

// Arduino Mega or Due - Serial1
// pin 19: RX - connect to ODrive TX
// pin 18: TX - connect to ODrive RX
// See https://www.arduino.cc/reference/en/language/functions/communication/serial/ for other options

HardwareSerial& odrive_serial = Serial1;

// ODrive object
ODriveArduino odrive(odrive_serial);

// Timer Variable
unsigned long programRunTime;
int sampPeriod = 1;

// SPI Pin
int pinCS = 53;

// ODrive Serial Variables
int motornum = 1;
int requested_state;

// Pushbutton Pin Variables (Digital)
boolean pbL = false; boolean pbR = false;
boolean pbM = false; 

boolean pbL_prev = false; boolean pbR_prev = false;
boolean pbM_prev = false;

// Trigger Variables
boolean pbLTrig = false; boolean pbRTrig = false;
boolean pbMTrig = false;

// Pushbutton Pins (Digital)
int pbL_dPin = 42; // Orange Wire [UP]
int pbR_dPin = 46; // Yellow Wire [DOWN]
int pbM_dPin = 44; // White Wire [TOGGLE]

// Load Cell Pin Variables (Analog)
int lcLeft; int lcRight;
int lcLeft_anPin = A3; int lcRight_anPin = A11;

// Limit Switch Values
boolean limitSwitchTop; boolean limitSwitchBottom;

// Limit Switch Pins
const int limitSwitchTop_dPin = 32; const int limitSwitchBottom_dPin = 36; 

// Digital Power Rail Pin
const int digPowerPin_ExpTrig = 27;   // 5V Source for External Trigger
const int digPowerPin_LLC = 30;       // 5V Source for Left Load Cell
const int digPowerPin_RLC = 31;       // 5V Source for Right Load Cell

// Control Variables
float posStep = 6; // Number of position steps to be added to the motor per button press 
float posTarget;
float pos;
boolean motorModeToggle = false;

// Error Variables
int motorErr = 1000; int controlErr = 1000; int encoderErr = 1000;
int axisErr = 1000; int sensorlessEstErr = 1000;

/* Max. Force Values for Load Cells */
// Max. Load (20% Muscle Weakness): 558 N = 125 lbsf, Sensor: 1119011G, Calibration Curve: V(f) = 0.0159*f-0.00224, Max. Signal @ 250 lbs:  3.969 Vdc
//int maxForceLeftLC = 406; // Expected Signal: 1.985V
int maxForceLeftLC = 204;

// Max. Load (20% Muscle Weakness): 558 N = 125 lbsf, Sensor: 11190020G , Calibration Curve: V(f) = 0.016*f-0.00552, Max. Signal @ 250 lbs: 3.998 Vdc
//int maxForceRightLC = 408; // Expected Signal: 1.994V
int maxForceRightLC = 204;

// System Monitoring Floats
float vbus;
float iQ_comm; float iQ_meas;
float vel_meas;
float iRegen; 

boolean motorStatus = 0; // 0 = IDLE; 1 = ARMED

int pbMCount; // 0 -> IDLE (RESET); 1 -> Full-Step Set; 2 -> Half-Step Set

void setup() {

  // ODrive uses 115200 baud
  odrive_serial.begin(115200);

  // Configure Pushbutton Control Pins
  pinMode(pbL_dPin, INPUT); pinMode(pbR_dPin, INPUT);
  pinMode(pbM_dPin, INPUT);

  // Configure Load Cell Pins
  pinMode(lcLeft_anPin, INPUT); pinMode(lcRight_anPin, INPUT);

  // Configure Limit Switch Pins
  pinMode(limitSwitchTop_dPin, INPUT); pinMode(limitSwitchBottom_dPin, INPUT);

  // Configure Pin for the Onboard LED
  pinMode(LED_BUILTIN, OUTPUT);

  // Turn On Digital Power Pins
  pinMode(digPowerPin_ExpTrig, OUTPUT); digitalWrite(digPowerPin_ExpTrig,HIGH);
  pinMode(digPowerPin_LLC, OUTPUT); digitalWrite(digPowerPin_LLC,HIGH);
  pinMode(digPowerPin_RLC, OUTPUT); digitalWrite(digPowerPin_RLC,HIGH);
  
  Serial.println("Pin configuration complete.");
  
  // Serial to PC
  Serial.begin(115200);
  Serial.println("Starting Serial Monitor...");
  while (!Serial); // wait for Arduino Serial Monitor to open
  Serial.println("Serial monitor connected...");

  Serial.println("Please ensure that the motor and encoder have been properly configured prior to running this script.");
  // The Arduino library seems to have configuration functions but for an initial script set configuration parameters directly using the ODriveTool

  // Start-up Sequence
  Serial.println("Please press any of the pushbuttons to start the program.");
  while((pbM == false) && (pbL == false) && (pbR == false)){
    pbL = digitalRead(pbL_dPin); pbR = digitalRead(pbR_dPin);
    pbM = digitalRead(pbM_dPin);
  }
  
  // Reboot the ODrive to reset the position to 0.00 and to clear error flags
  Serial.println("Resetting the ODrive motor controller.");
  odrive_serial.write("sr \n");
  delay(2000);
  pos = 0; posTarget = 0; pbMCount = 0;
  Serial.println("ODrive reboot successful.");

  // SD Card Intialization
  Serial.println("SD card initializing...");
  pinMode(pinCS, OUTPUT);
  Serial.println("CS Pin set.");
  
  if(SD.begin(pinCS) == true){
    Serial.println("SD card is ready for use.");
  }

  else{
    Serial.println("SD card initialization failed!");
    Serial.println("Potential Diagnostic Steps:");
    Serial.println("1. Check if SD card is inserted.");
    Serial.println("2. Double check SD Card Module wiring.");
    Serial.println("3. Ensure the chip select (CS) pin matches that designated for the SD card module.");
    Serial.println("Press reset or reopen this serial monitor after fixing the issue!");
    while (true);
  }

  // Check if directory for stored files exists
  if(SD.exists("TEST1030/MD_DATA/")){
    Serial.println("Directory exists and is ready for use.");
  }

  // Create directory if a directory for stored files does not yet exist on the SD card
  else{
    SD.mkdir("/TEST1030/MD_DATA/");
    Serial.println("Data directory created and ready for use.");
  }
  
  Serial.println("System Ready!");
  Serial.println("Starting program...");
  Serial.println("ODrive-Arudino Manual Motor Driving Script");
  
  // Code for inputs that result in motor movement
  Serial.println("To operate please do one of the following:");
  Serial.println("Press the left pushbutton to rotate the motor CCW (ball screw ascend).");
  Serial.println("Press the middle pushbutton to toggle the motor between IDLE and CLOSED_LOOP_CONTROL modes.");
  Serial.println("Press the right pushbutton to rotate the motor CW (ball screw descend.");
  
  /* Print initial lines to file */
  dataFile = SD.open("/TEST1030/MD_DATA/EXP#2_P6.txt", FILE_WRITE);
  
  // Write data labels to the SD card
  if (dataFile) {
    dataFile.println("System Ready!");
    dataFile.println("Beginning Experimental Trial...");
    dataFile.print("Time (ms)"); dataFile.print(",");
    dataFile.print("motPos_Comm"); dataFile.print(",");
    //dataFile.print("motPos_Meas"); dataFile.print(",");
    //dataFile.print("vel_Meas"); dataFile.print(",");
    //dataFile.print("iQ_Comm"); dataFile.print(","); dataFile.print("iQ_Meas"); dataFile.print(",");
    dataFile.print("lcLeft"); dataFile.print(","); dataFile.print("lcRight"); dataFile.print(",");
    dataFile.print("motorStatus"); dataFile.print(",");
    //dataFile.print("iRegen"); dataFile.print(",");
    //dataFile.print("motorErr"); dataFile.print(",");
    //dataFile.print("controlErr"); dataFile.print(",");
    //dataFile.print("encoderErr"); dataFile.print(",");
    //dataFile.print("axisErr"); dataFile.print(",");
    //dataFile.print("sensorlessEstErr"); Serial.print(",");
    //dataFile.print("pbMCount"); dataFile.print(",");
    dataFile.println();
    dataFile.close();

  }

  // Flag for error opening SD card
  else {
    Serial.println("Error opening write file!!");
  }
  
}


void loop() {

  // Record Time Stamp
  programRunTime = millis();

  // Record Push Button Readings
  pbL = digitalRead(pbL_dPin); pbR = digitalRead(pbR_dPin);
  pbM = digitalRead(pbM_dPin);

  // Set the Triggering Variables
  pbTrigSet();
  
  // Check Limit Switches
  limitSwitchTop = digitalRead(limitSwitchTop_dPin); limitSwitchBottom = digitalRead(limitSwitchBottom_dPin);

  /*
  // Set motor to IDLE Mode if Ball Screw Limit is Reached or if the Max. Force is Reached
  if( (limitSwitchTop == true) || (limitSwitchBottom == true) || (lcLeft >= maxForceLeftLC) || (lcRight >= maxForceRightLC)){
    requested_state = ODriveArduino::AXIS_STATE_IDLE;
    Serial << "Axis" << motornum << ": Requesting state " << requested_state << '\n';
    if(!odrive.run_state(motornum, requested_state, false)) return;
    Serial.println("Motor set to IDLE mode.");

    dataFile = SD.open("/TEST1030/MD_DATA/EXP#2_P6.txt", FILE_WRITE);
  */
  
    /*Debugging Code */
    /*
    if(dataFile && (limitSwitchTop == true)){
      dataFile.println("Top Limit Switch triggered.");
      dataFile.close();
    }

    else if(dataFile && (limitSwitchBottom == true)){
      dataFile.println("Bottom Limit Switch triggered");
      dataFile.close();
    }

    // Flag for error opening SD card
    else {
      dataFile.println("Error opening write file!!");
    }
    
  }
  */
  
  // Case 1: Toggle Arm/Disarm Motor
  if((pbMTrig == true) && (pbLTrig == false) && (pbRTrig == false)){
    
    motorModeToggle = toggleMotor();
    delay(1000);
  
  }

  // Case 2: Push Button Left Triggered (CCW Motor Rotation, Ball Screw Ascend)
  else if((pbMTrig == false) && (pbLTrig == true) && (pbRTrig == false) && (motorModeToggle == true)){

    // Single Step Conditional
    if(pbMCount == 1){
      posTarget = posTarget - posStep;
      odrive.TrapezoidalMove(motornum, posTarget); // Use for trapezoidal trajectory control modes
    }

    // Double Step Conditonal
    else{
      posTarget = posTarget - 2*posStep;
      odrive.TrapezoidalMove(motornum, posTarget); // Use for trapezoidal trajectory control modes
    }
    
  }

  // Case 3: Push Button Right Triggered (CW Motor Rotation, Ball Screw Descend)
  else if((pbMTrig == false) && (pbLTrig == false) && (pbRTrig == true) && (motorModeToggle == true)){

   // Single Step Conditional
   if(pbMCount == 1){
      posTarget = posTarget + posStep;
      odrive.TrapezoidalMove(motornum, posTarget); // Use for trapezoidal trajectory control modes
   }

    // Double Step Conditonal
    else{
      posTarget = posTarget + 2*posStep;
      odrive.TrapezoidalMove(motornum, posTarget); // Use for trapezoidal trajectory control modes
    }
    
  }

  // Case 4: No Input or No Valid Input
  else{
    // Do Nothing
  }

  // Record Load Cell Readings
  lcLeft = analogRead(lcLeft_anPin); lcRight = analogRead(lcRight_anPin);

  // Set the Previous Pushbutton Readings
  pbL_prev = pbL; pbR_prev = pbR;
  pbM_prev = pbM;

  // Record errors
  //errorRecord();

  if(motorStatus == 1){
    
    // Record sensor outputs and time to the SD card module
    recordSensorReads();
  
  }

  // Sampling Period Delay
  delay(sampPeriod);
  
}
