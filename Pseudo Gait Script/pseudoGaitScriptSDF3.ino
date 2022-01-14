/*
Chance Bisquera, cbisquera21@vt.edu
TMVS Lab
10/23/21

The following script is the main exoskeleton script that will be used when conducting gait experiments both with and without an actual user. This is an analytical
script, and is meant to help the user analyze an experiment prior to formally recording data to an SD card. The desired data is printed to the Arduino's serial monitor.
The data points of interest that are recorded at each MCU cycle include the analog signals of the 6 FSR sensors, the analog readings from the two load cell sensors, the
FSM gait state that the user is currently in for each leg, time, motor position, motor velocity, target motor current, and desired motor current.

For data collection purposes, this script records the analog readings from the 6 FSR sensors, two load cell sensor, and the FSM state that the user is in for
each leg.

Intended MCU: Arduino Mega 2560
SD Card Module used:
https://www.amazon.com/KeeYees-Micro-Adater-Reader-Module/dp/B07QQBBL6S/ref=sr_1_3?dchild=1&keywords=micro+sd+board+arduino&qid=1622227882&s=electronics&sr=1-3

Note:
-This script requires an external trigger circuit with pushbuttons and does not require an LED to indicate data logging status. The status of data recording is
indicated by the Arduino's onboard LED.

File Naming
FAT file systems have a limitation when it comes to naming conventions. You must use the 8.3 format, so that file names look like “NAME001.EXT”, 
where “NAME001” is an 8 character or fewer string, and “EXT” is a 3 character extension. People commonly use the extensions .TXT and .LOG. It is 
possible to have a shorter file name (for example, mydata.txt, or time.log), but you cannot use longer file names. Read more on the 8.3 convention.

-Reference: https://www.arduino.cc/en/Reference/SDCardNotes

References:
-https://www.youtube.com/watch?v=5Dp-XatLySM&list=LL&index=4&t=96s
-https://www.arduino.cc/en/Tutorial/LibraryExamples/Datalogger
-https://www.arduino.cc/en/reference/SD

*/

#include<SD.h>
#include<SPI.h>

#include <HardwareSerial.h>
#include <SoftwareSerial.h>
#include <ODriveArduino.h>

File dataFile; 

// SPI Pin
int pinCS = 53;

// Printing with stream operator helper functions
template<class T> inline Print& operator <<(Print &obj,     T arg) { obj.print(arg);    return obj; }
template<>        inline Print& operator <<(Print &obj, float arg) { obj.print(arg, 4); return obj; }

////////////////////////////////
// Set up serial pins to the ODrive
// *Don't forget to also connect ODrive GND to Arduino GND. (if the ODrive and Arduino run on two different power sources would it be non-ideal to connect GND)?
////////////////////////////////

// Arduino Mega or Due - Serial1
// pin 19: RX - connect to ODrive TX (GPIO Pin 1 -> UART_A)
// pin 18: TX - connect to ODrive RX (GPIO Pin 2 -> UART_A)
// See https://www.arduino.cc/reference/en/language/functions/communication/serial/ for other options
// HardwareSerial& odrive_serial = Serial1;

HardwareSerial& odrive_serial = Serial1;

// ODrive object
ODriveArduino odrive(odrive_serial);

// ODrive Serial Variables
int motornum = 1;
int requested_state;

// Timer Variables
unsigned long programRunTime;
int sampPeriod = 1;
int actTrigState = 2; // Trigger state for the exoskeleton actuator (trigger occurs when a transition from the previous state to actTrig is detected)

// Trigger Buffer
int trigBuffer;

// FSR Output Variables
int fsrL1_MT;   // Left Foot: medial, top (Blue Wire, A0)
int fsrL2_LT;   // Left Foot: lateral, top (Green Wire, A1)
int fsrL3_H;    // Left Foot: heel (Purple Wire, A2)

int fsrR1_MT;   // Right Foot: medial, top (Red Wire, A8)
int fsrR2_LT;   // Right Foot: lateral, top (Orange Wire, A9)
int fsrR3_H;    // Right Foot: heel (Yellow Wire, A10)

int fsr12SumL = 0; int fsr12SumR = 0;   // Net FSR Outputs for FSRs around the MTP Joints
int fsrNetL = 0; int fsrNetR = 0;       //Net FSR Outputs

// FSR Pins
const int fsrL1_anPin = A0; const int fsrL2_anPin = A1; const int fsrL3_anPin = A2;
const int fsrR1_anPin = A8; const int fsrR2_anPin = A9; const int fsrR3_anPin = A10; // Configuration right FSRs on an Arduino Mega

// Load Cell Output Variables
int lcLeft;   // Left Load Cell
int lcRight;  // Right Load Cell

// Load Cell Pins
const int lcLeft_anPin = A3; const int lcRight_anPin = A11;

// Limit Switch Values
boolean limitSwitchTop; boolean limitSwitchBottom;

// Limit Switch Pins
const int limitSwitchTop_dPin = 32; const int limitSwitchBottom_dPin = 36; 

// Digital Power Rail Pin
const int digPowerPin_ExpTrig = 27;   // 5V Source for External Trigger
const int digPowerPin_LLC = 30;       // 5V Source for Left Load Cell
const int digPowerPin_RLC = 31;       // 5V Source for Right Load Cell
const int digPowerPin_FSR_L1 = 38;    // 5V Source for FSR L1 (Medial, Anterior FSR)
const int digPowerPin_FSR_L2 = 40;    // 5V Source for FSR L2 (Lateral, Anterior FSR)
const int digPowerPin_FSR_R1 = 39;    // 5V Source for FSR R1 (Medial, Anterior FSR)
const int digPowerPin_FSR_R2 = 41;    // 5V Source for FSR R2 (Lateral, Anterior FSR)

// Detection Variables
boolean step1Check;
boolean hsDetectLeft = false; boolean hsDetectRight = false;
boolean actTrigLeft = false; boolean actTrigRight = false;
char lastTrig;

// Experimental Trigger Switch Variables
int expTrigPin = 42;    // Orange Wire
int resetIdlePin = 44;  // White Wire

//boolean resetIdle = false; 
boolean pbETrig = false; 
boolean expTrig = false; boolean lastExpTrig = false;

// Motor Position Variable
float pos;
float posStep;
float posStepHalf = 6;
float posStepFull = 2*posStepHalf;
//float pos_OD; // ODrive motor position control

/* Max. Force Values for load cells */
// Max. Load (20% Muscle Weakness): 558 N = 125 lbsf, Sensor: 1119011G, Calibration Curve: V(f) = 0.0159*f-0.00224, Max. Signal @ 250 lbs:  3.969 Vdc
//int maxForceLeftLC = 406; // Expected Signal: 1.985V
int maxForceLeftLC = 204;

// Max. Load (20% Muscle Weakness): 558 N = 125 lbsf, Sensor: 11190020G , Calibration Curve: V(f) = 0.016*f-0.00552, Max. Signal @ 250 lbs: 3.998 Vdc
//int maxForceRightLC = 408; // Expected Signal: 1.994V
int maxForceRightLC = 204;

// System Monitoring Floats
float vbus;
float iRegen; 

float vel_meas;
float iQ_comm; float iQ_meas;

// FSM State Tracker Variables
int fsmStateLeft = 0; int fsmStateRight = 0;
int fsmStateLeft_prev = 0; int fsmStateRight_prev = 0;

// Debugging Variables
int motorCond = 0;

/*
 For the sake of programming simplicity the typical phases constituting the gait cycle were adapted to 
 4 different gait cycle States that can be identified based on the output signal patterns for the FSR 
 array lining each foot.

    State 1 -> Heelstrike to Loading Response (0-12% of Gait Cycle)
    State 2 -> Midstance to Early Terminal Stance (12-~40.5% of Gait Cycle)
    State 3 -> Late Terminal Stance to Pre-Swing (~40.5-62% of Gait Cycle)
    State 4 -> Swing Period (62-100% of Gait Cycle)

  FSM States:
    S0 -> Unknown State
    S1 -> State 1
    S2 -> State 2
    S3 -> State 3
    S4 -> State 4
    S -1 -> Both feet on ground
    S -2 -> Both feet off ground
*/

void setup() {

  // ODrive uses 115200 baud
  odrive_serial.begin(115200);

  // Serial to PC
  Serial.begin(115200);
  Serial.println("Starting Serial Monitor...");
  while (!Serial) ; // wait for Arduino Serial Monitor to open
  Serial.println("Serial monitor connected...");
  
  Serial.println("Program starting...");

  // Set FSR Pins
  Serial.println("Starting pin configuration...");

  // Configure FSR Pins
  pinMode(fsrL1_anPin, INPUT); pinMode(fsrL2_anPin, INPUT); pinMode(fsrL3_anPin, INPUT);
  pinMode(fsrR1_anPin, INPUT); pinMode(fsrR2_anPin, INPUT); pinMode(fsrR3_anPin, INPUT);

  // Configure Load Cell Pins
  pinMode(lcLeft_anPin, INPUT); pinMode(lcRight_anPin, INPUT);
  
  // Configure Limit Switch Pins
  pinMode(limitSwitchTop_dPin, INPUT); pinMode(limitSwitchBottom_dPin, INPUT);

  // Configure Trigger Switch Circuit Pins
  pinMode(expTrigPin, INPUT); //pinMode(resetIdlePin, INPUT); 

  // Turn On Digital Power Pins
  pinMode(digPowerPin_ExpTrig, OUTPUT); digitalWrite(digPowerPin_ExpTrig,HIGH);
  pinMode(digPowerPin_LLC, OUTPUT); digitalWrite(digPowerPin_LLC,HIGH);
  pinMode(digPowerPin_RLC, OUTPUT); digitalWrite(digPowerPin_RLC,HIGH);
  pinMode(digPowerPin_FSR_L1, OUTPUT); digitalWrite(digPowerPin_FSR_L1,HIGH);
  pinMode(digPowerPin_FSR_L2, OUTPUT); digitalWrite(digPowerPin_FSR_L2,HIGH);
  pinMode(digPowerPin_FSR_R1, OUTPUT); digitalWrite(digPowerPin_FSR_R1,HIGH);
  pinMode(digPowerPin_FSR_R2, OUTPUT); digitalWrite(digPowerPin_FSR_R2,HIGH);

  // Set On-Board LED Pin
  pinMode(LED_BUILTIN, OUTPUT);

  // Reset the tracker variable for the last trigger variable
  lastTrig = 'N';
    
  Serial.println("Pin configuration complete.");

  pos = 0; // Set the starting position of the motor to 0 following reboot

  step1Check = false; // Reset the first step indicator following reboot

  trigBuffer = 0; // Reset the gait trigger buffer following reboot

  // Activate the motor by pushing the experimental trigger pushbutton (Initiate Start-Up)
  Serial.println("Please press the experiment trigger pushbutton to arm the motor.");
  while(pbETrig == false){
    pbETrig = digitalRead(expTrigPin);
  }

  delay(2000);

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
  if(SD.exists("/TEST1026/PSGT_EXP")){
    Serial.println("Directory exists and is ready for use.");
  }

  // Create directory if a directory for stored files does not yet exist on the SD card
  else{
    SD.mkdir("/TEST1026/PSGT_EXP");
    Serial.println("Data directory created and ready for use.");
  }
  
  /* Print initial lines to file */
  dataFile = SD.open("/TEST1026/PSGT_EXP/EXP#5.txt", FILE_WRITE);
  
  Serial.println("Starting main program...");

  // Write time stamp and sensor output to file if file is available
  if (dataFile) {
    dataFile.println("New experimental run starting...");
    dataFile.print("Sample Rate = "); dataFile.print(1/(sampPeriod*0.001)); dataFile.print(" Hz");
    dataFile.println();
    dataFile.print("Time (ms)"); dataFile.print(",");
    dataFile.print("FSR L1"); dataFile.print(","); dataFile.print("FSR L2"); dataFile.print(","); dataFile.print("FSR L3"); dataFile.print(",");
    dataFile.print("FSR R1"); dataFile.print(","); dataFile.print("FSR R2"); dataFile.print(","); dataFile.print("FSR R3"); dataFile.print(",");
    dataFile.print("fsmStateLeft"); dataFile.print(","); dataFile.print("fsmStateRight"); dataFile.print(","); 
    //dataFile.print("lcLeft"); dataFile.print(","); dataFile.print("lcRight"); dataFile.print(",");
    dataFile.print("motorPos"); dataFile.print(","); dataFile.print("posStep"); dataFile.print(",");
    //dataFile.print("hsDetectLeft"); dataFile.print(","); dataFile.print("hsDetectRight"); dataFile.print(",");
    //dataFile.print("limitSwitchTop"); dataFile.print(","); dataFile.print("limitSwitchBottom"); dataFile.print(",");
    dataFile.print("actTrigLeft"); dataFile.print(","); dataFile.print("actTrigRight"); dataFile.print(",");
    //dataFile.print("step1Check"); dataFile.print(","); 
    dataFile.print("lastTrig"); dataFile.print(",");
    dataFile.print("expTrig"); dataFile.print(","); dataFile.print("lastExpTrig"); dataFile.print(",");
    dataFile.print("motorCond"); dataFile.print(",");
    dataFile.print("trigBuffer"); dataFile.print(",");
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
  
  // Record Outputs of FSR Sensors
  fsrL1_MT = analogRead(fsrL1_anPin); fsrL2_LT = analogRead(fsrL2_anPin); fsrL3_H = analogRead(fsrL3_anPin);
  fsrR1_MT = analogRead(fsrR1_anPin); fsrR2_LT = analogRead(fsrR2_anPin); fsrR3_H = analogRead(fsrR3_anPin);

  fsr12SumL = fsrL1_MT + fsrL2_LT; fsr12SumR = fsrR1_MT + fsrR2_LT;
  fsrNetL = fsrL1_MT + fsrL2_LT + fsrL3_H; fsrNetR = fsrR1_MT + fsrR2_LT + fsrR3_H;

  // Set the value for the experimental trigger
  expTrigSet();

  /* Main FSM Code */
  // State L1: Left Foot State 1
  if((fsrL3_H > 511) && (fsr12SumL < 205)){
    fsmStateLeft = 1;
  }

  // State L2: Left Foot State 2
  else if((fsrL3_H >= 819) && (fsr12SumL >= 205)){
    fsmStateLeft = 2;
  }

  // State L3: Left Foot State 3
  else if((fsrL3_H < 819) && (fsr12SumL >= 205)){
    fsmStateLeft = 3;
  }

  // State L4: Left Foot State 4
  else if((fsrL3_H < 819) && (fsr12SumL < 205) && (fsrNetR >= 409)){
    fsmStateLeft = 4;
  }

  // State L0: Unknown Left Foot State
  else{
    fsmStateLeft = 0;
  }

  // State R1: Right Foot State 1
  if((fsrR3_H > 511) && (fsr12SumR < 205)){
    fsmStateRight = 1;
  }

  // State R2: Right Foot State 2
  else if((fsrR3_H >= 819) && (fsr12SumR >= 205)){
    fsmStateRight = 2;
  }

  // State R3: Right Foot State 3
  else if((fsrR3_H < 819) && (fsr12SumR >= 205)){
    fsmStateRight = 3;
  }

  // State R4: Right Foot State 4
  else if((fsrR3_H < 819) && (fsr12SumR < 205) && (fsrNetL >= 409)){
    fsmStateRight = 4;
  }

  // State R5: Unknown Right Foot State
  else{
    fsmStateRight = 0;
  }

  // State -1: Both Feet on Ground (User Standing) 
  if( (fsr12SumL >= 1228) && (fsr12SumR >= 1228) && (fsrL3_H >= 871) && (fsrR3_H >= 871) ){
    fsmStateLeft = -1; fsmStateRight = -1;
  }

  // State -2: Both Feet off the Ground
  //if((fsrNetL <= 409) && (fsrNetR <= 409)){
  if((fsrNetL <= 205) && (fsrNetR <= 205)){
    fsmStateLeft = -2; fsmStateRight = -2;
  }

  /* Motor Control Script */
  // Record Load Cell Readings
  //lcLeft = analogRead(lcLeft_anPin); lcRight = analogRead(lcRight_anPin);

  // Check Limit Switches
  //limitSwitchTop = digitalRead(limitSwitchTop_dPin); limitSwitchBottom = digitalRead(limitSwitchBottom_dPin);

  // Check the Neutral Reset Toggle Switch
  //resetIdle = digitalRead(resetIdlePin);

  /* Trigger Functions */
  //heelStrikeDetect(); // Detect if a heelstrike has taken place (comment out if State 2 onset is trigger)
  phaseDetect234(actTrigState); // Detect if the triggering gait transition has taken place (comment out if State 1 onset is trigger)

  // Set the value for posStep
  if((trigBuffer <= 4) && (expTrig == true)){
    posStep = posStepHalf;
  }

  else{
    posStep = posStepFull;
  }
  
  /* Motor Movement Code */
  // Reset the actuator to the idle neutral position if pos > 0
  if((expTrig == false) && (lastExpTrig == true) && (pos > 0)){
    motorCond = 1;
    //pos = pos - posStep;
    //Serial.println("Moving the exoskeleton actuator to its neutral position...");
    step1Check = false;
    lastTrig = 'N';
    //trigBuffer = 0;
  }

  // Reset the actuator to the idle neutral position if pos < 0
  else if((expTrig == false) && (lastExpTrig == true) && (pos < 0)){
    motorCond = 2;
    //pos = pos + posStep;
    //Serial.println("Moving the exoskeleton actuator to its neutral position...");
    step1Check = false;
    lastTrig = 'N';
    //trigBuffer = 0;
  }

  // Left Foot in Stance Phase (CCW Motor Rotation, Ball Screw Descends)
  else if((actTrigLeft == true) && (actTrigRight == false) && (expTrig == true) && (trigBuffer >= 4)){
    motorCond = 3;
    pos = pos + posStep;
  }

  // Right Foot in Stance Phase (CW Motor Rotation, Ball Screw Ascends)
  else if((actTrigLeft == false) && (actTrigRight == true) && (expTrig == true) && (trigBuffer >= 4)){
    motorCond = 4;
    pos = pos - posStep;
  }
  
  // User not in a Normal Gait Cycle (i.e. user standing, has both feet off the ground, or is in an unknown state)
  else{
    // Do nothing
    motorCond = 0;
  }
   
  /* End Cycle Operations */
  if(expTrig == true){
    Serial.println("Recording Data to SD Card.");
    dataFile = SD.open("/TEST1026/PSGT_EXP/EXP#5.txt", FILE_WRITE);
    
    // Write time stamp and sensor output to file if file is available
    if (dataFile) {
      // Print readings to the SD card
      dataFile.print(programRunTime); dataFile.print(",");
      dataFile.print(fsrL1_MT); dataFile.print(","); dataFile.print(fsrL2_LT); dataFile.print(","); dataFile.print(fsrL3_H); dataFile.print(",");
      dataFile.print(fsrR1_MT); dataFile.print(","); dataFile.print(fsrR2_LT); dataFile.print(","); dataFile.print(fsrR3_H); dataFile.print(",");
      dataFile.print(fsmStateLeft); dataFile.print(","); dataFile.print(fsmStateRight); dataFile.print(",");
      //dataFile.print(lcLeft); dataFile.print(","); dataFile.print(lcRight); dataFile.print(",");
      dataFile.print(pos); dataFile.print(","); dataFile.print(posStep); dataFile.print(",");
      //dataFile.print(hsDetectLeft); dataFile.print(","); dataFile.print(hsDetectRight); dataFile.print(",");
      //dataFile.print(limitSwitchTop); dataFile.print(","); dataFile.print(limitSwitchBottom); dataFile.print(",");
      dataFile.print(actTrigLeft); dataFile.print(","); dataFile.print(actTrigRight); dataFile.print(",");
      //dataFile.print(step1Check); dataFile.print(","); 
      dataFile.print(lastTrig); dataFile.print(",");
      dataFile.print(expTrig); dataFile.print(","); dataFile.print(lastExpTrig); dataFile.print(",");
      dataFile.print(motorCond); dataFile.print(",");
      dataFile.print(trigBuffer); dataFile.print(",");
      dataFile.println();
      dataFile.close();
    }

    // Flag for error opening SD card
    else {
      Serial.println("Error opening write file!!");
    }
  
  }
  
  // Print Readings to Serial Monitor
  //printSensorReads();

  // Record the previous experimental trigger value
  lastExpTrig = expTrig;
  
  // Set the previous gait cycle States in preparation for the next MCU cycle
  fsmStateLeft_prev = fsmStateLeft; fsmStateRight_prev = fsmStateRight;
  
  // Intentional delay for sampling
  delay(sampPeriod);

}
