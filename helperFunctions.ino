/*

  This script is used for the functions referenced in the main "gaitFSM.ino" script that help the FSM operate appropriately.

  Note: The Arduino Mega ADC is 10 bits, meaning that voltage conversions are done in acccordance with the ratios:
    k = 5V/1024 and invk = 1/k = 1024/5V

    203 -> 1V; 306 -> 1.5V; 409 -> 2V; 613 -> 3V; 715 -> 3.5V; 818 -> 4V; 921 -> 4.5V; 1023 -> 5V; 1228 -> 6V; 
    1433 -> 7V; 2763 -> 13.5V
    
*/

// Function for detecting heelstrike
void heelStrikeDetect(){

  // Detect heelstrike in left foot
  if((fsmStateLeft == 1) && (fsmStateLeft_prev == 4) && (lastTrig != 'L') && (expTrig == true)){
    
    hsDetectLeft = true;
    hsDetectRight = false;
    
    actTrigLeft = true;
    actTrigRight = false;
    lastTrig = 'L';
    trigBuffer++;

  }

  // Detect heelstrike in right foot
  else if((fsmStateRight == 1) && (fsmStateRight_prev == 4) && (lastTrig != 'R') && (expTrig == true)){
    
    hsDetectLeft = false;
    hsDetectRight = true;
    
    actTrigLeft = false;
    actTrigRight = true;
    lastTrig = 'R';
    trigBuffer++;

  }

  //
  else if((fsmStateLeft < 1) || (fsmStateRight < 1)){
    
    hsDetectLeft = false;
    hsDetectRight = false;
    
    actTrigLeft = false;
    actTrigRight = false;

  }

  else{

    actTrigLeft = false;
    actTrigRight = false;

  }
  
}

// Function for setting the experimental trigger to record and stop recording data
void expTrigSet(){

  pbETrig = digitalRead(expTrigPin);

  // Set the experimental trigger variable to true (DATA RECORDING ON)
  if((pbETrig == true) && (expTrig == false)){
    
    digitalWrite(LED_BUILTIN, HIGH);
    expTrig = true;
    delay(1000); // Debouncing delay
    trigBuffer = 0;
    pos = 0;

    Serial.println("Arming motor...");
    requested_state = ODriveArduino::AXIS_STATE_CLOSED_LOOP_CONTROL;
    Serial << "Axis" << motornum << ": Requesting state " << requested_state << '\n';
    if(!odrive.run_state(motornum, requested_state, false)) return;
    Serial.println("Motor Armed!");
    
    dataFile = SD.open("/TEST1102/MAIN_EXP/EXP#1.txt", FILE_WRITE);
    
    // Data Recording Start Flag
    if (dataFile) {
      // Print readings to the SD card
      dataFile.println("Recording starting...");
      dataFile.close();
    }
    
    // Flag for error opening SD card
    else {
      Serial.println("Error opening write file!!");
    }
    
    delay(1000);
  
  }

  // Set the experimental trigger variable to false (DATA RECORDING OFF)
  else if((pbETrig == true) && (expTrig == true)){

    digitalWrite(LED_BUILTIN, LOW);
    expTrig = false;
    trigBuffer = 0;
    lastTrig = 'N';
    
    dataFile = SD.open("/TEST1102/MAIN_EXP/EXP#1.txt", FILE_WRITE);
    
    // Data Recording Termination Flag
    if (dataFile) {
      dataFile.println("Recording ceased!");
      dataFile.println("***************************************************************");
      dataFile.close();

      requested_state = ODriveArduino::AXIS_STATE_IDLE;
      Serial << "Axis" << motornum << ": Requesting state " << requested_state << '\n';
      if(!odrive.run_state(motornum, requested_state, false)) return;
      Serial.println("Motor set to IDLE mode.");

      
    }
    
    // Flag for error opening SD card
    else {
      Serial.println("Error opening write file!!");
    }
    
    delay(1000);
    
  }

  else{
    // do nothing
  }

}

// Function for detecting gait cycle phases 2, 3, or 4
void phaseDetect234(int phase){

  // Detect the trigger in the left foot
  if((fsmStateLeft == phase) && (fsmStateLeft_prev == phase-1) && (lastTrig != 'L') && (expTrig == true)){
    actTrigLeft = true;
    actTrigRight = false;
    lastTrig = 'L';
    trigBuffer++;
  }

  // Detect the trigger in the right foot
  else if((fsmStateRight == phase) && (fsmStateRight_prev == phase-1) && (lastTrig != 'R') && (expTrig == true)){
    actTrigLeft = false;
    actTrigRight = true;
    lastTrig = 'R';
    trigBuffer++;
  }

  //
  else if((fsmStateLeft < 1) || (fsmStateRight < 1)){
    actTrigLeft = false;
    actTrigRight = false;
  }

  else{
    actTrigLeft = false;
    actTrigRight = false;
  }
  
}

// Function for recording motor errors
void errorDetect(){

  // Print Errors to the SD card when the user manually detects a motor stop
  if((errTrig == true) && (errTrig_prev == false)){

    odrive_serial.write("r axis1.motor.error\n");
    motorErr = odrive.readInt();

    odrive_serial.write("r axis1.controller.error\n");
    controlErr = odrive.readInt();

    odrive_serial.write("r axis1.encoder.error\n");
    encoderErr = odrive.readInt();

    odrive_serial.write("r axis1.error\n");
    axisErr = odrive.readInt();

    odrive_serial.write("r axis1.sensorless_estimator.error\n");
    sensorlessEstErr = odrive.readInt();

    dataFile = SD.open("/TEST1102/MAIN_EXP/EXP#1.txt", FILE_WRITE);

    // Write time stamp and sensor output to file if file is available
    if (dataFile) {
      
      dataFile.println("Detected ODrive Errors: ");
      dataFile.print("motorErr = "); dataFile.print(motorErr); dataFile.println();
      dataFile.print("controlErr = "); dataFile.print(controlErr); dataFile.println();
      dataFile.print("encoderErr = "); dataFile.print(encoderErr); dataFile.println();
      dataFile.print("axisErr = "); dataFile.print(axisErr); dataFile.println();
      dataFile.print("sensorlessEstErr = "); dataFile.print(sensorlessEstErr); dataFile.println();
      dataFile.println();
      dataFile.close();
    
    }
  
    // Flag for error opening SD card
    else {
      Serial.println("Error opening write file!!");
    }
  
    delay(2000);
    
  }

  else{
    // Do nothing
  }
  
}
