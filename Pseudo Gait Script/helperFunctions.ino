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

  // Reset triggers if an invalid state is detected
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
    
    dataFile = SD.open("/TEST1026/PSGT_EXP/EXP#5.txt", FILE_WRITE);
    
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
    
    dataFile = SD.open("/TEST1026/PSGT_EXP/EXP#5.txt", FILE_WRITE);
    
    // Data Recording Termination Flag
    if (dataFile) {
      dataFile.println("Recording ceased!");
      dataFile.println("***************************************************************");
      dataFile.close();
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

// Function for printing the sensor outputs to the serial monitor
void printSensorReads(){
  
  Serial.print(programRunTime); Serial.print(",");
  Serial.print(fsrL1_MT); Serial.print(","); Serial.print(fsrL2_LT); Serial.print(","); Serial.print(fsrL3_H); Serial.print(",");
  Serial.print(fsrR1_MT); Serial.print(","); Serial.print(fsrR2_LT); Serial.print(","); Serial.print(fsrR3_H); Serial.print(",");
  Serial.print(fsmStateLeft); Serial.print(","); Serial.print(fsmStateRight); Serial.print(",");
  Serial.print(lcLeft); Serial.print(","); Serial.print(lcRight); Serial.print(",");
  Serial.print(pos); Serial.print(","); //Serial.print(pos_OD); Serial.print(",");
  Serial.print(hsDetectLeft); Serial.print(","); Serial.print(hsDetectRight); Serial.print(",");
  Serial.print(limitSwitchTop); Serial.print(","); Serial.print(limitSwitchBottom); Serial.print(",");
  Serial.print(actTrigLeft); Serial.print(","); Serial.print(actTrigRight); Serial.print(",");
  Serial.print(step1Check); Serial.print(","); Serial.print(lastTrig); Serial.print(",");
  //Serial.print(resetIdle); Serial.print(","); Serial.print(expTrig); Serial.print(",");
  Serial.print(motorCond); Serial.print(","); Serial.print("posStep"); Serial.print(",");
  Serial.println();
  
}
