/*

  This script contains the helper functions referenced in the main "manualMotorControl.ino" script.
    
*/

// Function for setting the motor's operational state between IDLE and CLOSED_LOOP_CONTROL
boolean toggleMotor(){
  
  // IDLE -> ACTIVE HALF-STEP (pbMCount == 1)
  if((motorModeToggle == false) && (pbMCount == 1)){
    
    requested_state = ODriveArduino::AXIS_STATE_CLOSED_LOOP_CONTROL;
    Serial << "Axis" << motornum << ": Requesting state " << requested_state << '\n';
    if(!odrive.run_state(motornum, requested_state, false)) return;
    
    motorStatus = 1;
    digitalWrite(LED_BUILTIN,HIGH);

    /* Indicate that data recording has started */
    dataFile = SD.open("/TEST1030/MD_DATA/EXP#2_P6.txt", FILE_WRITE);
        
    // Write data labels to the SD card
    if (dataFile) {
      dataFile.println("Data Recording Starting...");
      dataFile.close();  
    }
  
    // Flag for error opening SD card
    else {
      Serial.println("Error opening write file!!");
    }
    
    //Serial.println("Motor armed.");

    return true;
    
  }

  // ACTIVE HALF-STEP -> ACTIVE FULL-STEP (pbMCount == 2)
  else if((motorModeToggle == true) && (pbMCount == 2)){

    motorStatus = 1;
    digitalWrite(LED_BUILTIN,HIGH);
    return true;
    
  }

  // ACTIVE FULL-STEP -> IDLE (pbMCount == 0)
  else{

    requested_state = ODriveArduino::AXIS_STATE_IDLE;
    Serial << "Axis" << motornum << ": Requesting state " << requested_state << '\n';
    if(!odrive.run_state(motornum, requested_state, false)) return;
    
    motorStatus = 0;
    digitalWrite(LED_BUILTIN,LOW);
    
    //Serial.println("Motor set to IDLE mode.");
    
    //Serial.println("Resetting the ODrive motor controller.");
    odrive_serial.write("sr \n");
    delay(2000);
    pos = 0;  posTarget = 0;
    //Serial.println("ODrive reboot successful.");
    
    return false;
    
  }
  
}

// Function for setting the experimental trigger to record and stop recording data
void pbTrigSet(){

  // Set the value for the Left Pushbutton
  if((pbL == true) && (pbL_prev == false)){
    pbLTrig = true;
  }

  else{
    pbLTrig = false;
  }

  // Set the value for the Middle Pushbutton
  if((pbM == true) && (pbM_prev == false)){
    
    pbMTrig = true;
    pbMCount++; 

    // Reset pbMCount
    if(pbMCount > 2){
      pbMCount = 0;
    }
    
  }

  else{
    pbMTrig = false;
  }
  
  // Set the value for the Right Pushbutton
  if((pbR == true) && (pbR_prev == false)){
    pbRTrig = true;
  }

  else{
    pbRTrig = false;
  }

}


void errorRecord(){

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

}


// Function for printing the sensor outputs to the serial monitor
void recordSensorReads(){

  // Print and record position and measured and intended velocity and currents
  //odrive_serial.write("r axis1.encoder.pos_estimate\n");
  //pos = odrive.readFloat();

  //odrive_serial.write("r axis1.encoder.vel_estimate\n");
  //vel_meas = odrive.readFloat();

  //odrive_serial.write("r axis1.motor.current_control.Iq_setpoint\n");
  //iQ_comm = odrive.readFloat();
  //odrive_serial.write("r axis1.motor.current_control.Iq_measured\n");
  //iQ_meas = odrive.readFloat();

  //odrive_serial.write("r ibus\n");
  //iRegen = odrive.readFloat();

  /* Record Data to SD Card */
  dataFile = SD.open("/TEST1030/MD_DATA/EXP#2_P6.txt", FILE_WRITE);

  // Write data labels to the SD card
  if (dataFile){
    
    dataFile.print(programRunTime); dataFile.print(",");
    dataFile.print(posTarget); dataFile.print(","); 
    //dataFile.print(pos); dataFile.print(",");
    //dataFile.print(vel_meas); dataFile.print(",");
    //dataFile.print(iQ_comm); dataFile.print(","); dataFile.print(iQ_meas); dataFile.print(",");
    dataFile.print(lcLeft); dataFile.print(","); dataFile.print(lcRight); dataFile.print(",");
    dataFile.print(motorStatus); dataFile.print(",");
    //dataFile.print(iRegen); dataFile.print(",");
    //dataFile.print(motorErr); dataFile.print(",");
    //dataFile.print(controlErr); dataFile.print(",");
    //dataFile.print(encoderErr); dataFile.print(",");
    //dataFile.print(axisErr); dataFile.print(",");
    //dataFile.print(sensorlessEstErr); dataFile.print(",");
    //dataFile.print(pbMCount); dataFile.print(",");
    dataFile.println();
    dataFile.close();

  }

  // Flag for error opening SD card
  else {
    
    Serial.println("Error opening write file!!");
  
  }

}
