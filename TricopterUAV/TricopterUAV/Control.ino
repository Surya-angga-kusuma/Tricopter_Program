void control()
{
//  deltaTime_control_rate = millis() - lastTime_control_rate;
//
//  if(deltaTime_control_rate >= 10)
//  {
	// ========== roll control ==========
	errorRoll = roll_deg - roll_input;
	ControllerIRoll += GainIroll * (errorRoll);
	if (ControllerIRoll > maxController){ ControllerIRoll = maxController; }
	else if (ControllerIRoll < maxController * -1){ ControllerIRoll = maxController * -1; }

//    ControlRoll = GainProll * errorRoll + ControllerIRoll + GainDroll * (errorRoll - ControllerDRollLast);
	ControlRoll = (GainProll * errorRoll) + ControllerIRoll + (GainDroll * gx);
//  ControlRoll = 0;
  
	if (ControlRoll > maxController){ ControlRoll = maxController; }
	else if (ControlRoll < maxController * -1){ ControlRoll = maxController * -1; }
	ControllerDRollLast = errorRoll;

	// ========== pitch control ==========
	errorPitch = gyro_pitch_input - pitch_input;
	ControllerIPitch += GainIpitch * (errorPitch/1.321940560625);
	if (ControllerIPitch > maxController){ ControllerIPitch = maxController; }
	else if (ControllerIPitch < maxController * -1){ ControllerIPitch = maxController * -1; }
 
//	ControlPitch = GainPpitch * (errorPitch/1.321940560625) + ControllerIPitch + GainDpitch * ((errorPitch - ControllerDPitchLast)/1.321940560625);
  ControlPitch = 0;
 
	if (ControlPitch > maxController){ ControlPitch = maxController; }
	else if (ControlPitch < maxController * -1){ ControlPitch = maxController * -1; }
	ControllerDPitchLast = errorPitch;

	// ========== yaw control ==========
	errorYaw = gyro_yaw_input - yaw_input;
	ControllerIYaw += GainIyaw * (errorYaw*5.482456140625);
	if (ControllerIYaw > maxController){ ControllerIYaw = maxController; }
	else if (ControllerIYaw < maxController * -1){ ControllerIYaw = maxController * -1; }
 
//	ControlYaw = GainPyaw * errorYaw + ControllerIYaw + GainDyaw * (errorYaw - ControllerDYawLast);
    ControlYaw = 0;
 
	if (ControlYaw > maxController){ ControlYaw = maxController; }
	else if (ControlYaw < maxController * -1){ ControlYaw = maxController * -1; }
	ControllerDYawLast = errorYaw;

//  lastTime_control_rate = millis();
//  }
}

void control_update()
{
  if(ch5==0)
  {
    yawDegrees = headingDegrees ;
    heading_control = 0;
    throttle = 1000;
    
    ControllerIRoll = 0;
    ControllerIPitch = 0;
    ControllerIYaw = 0;
    ControllerDRollLast = 0;
    ControllerDPitchLast = 0;
    ControllerDYawLast = 0;
    
    servo    = servoAngleInit;
    motor1   = 1000;
    motor2   = 1000;
    motor3   = 1000;
  }

  if(ch5==1)
  {
    heading_control = yawDegrees - headingDegrees;
    throttle_input = throttle_channel;
    
    if (throttle_input > 1700)
		{
			throttle_input = 1700;
		}
   
    control();
    
    motor1 = throttle_input - ControlRoll + (ControlPitch*2/3) ; //RIGHT
    motor2 = throttle_input + ControlRoll + (ControlPitch*2/3) ; //LEFT
    motor3 = throttle_input - (ControlPitch*4/3);                //REAR
    servo = constrain(servoAngleInit + (ControlYaw*-1), (servoAngleInit-45), (servoAngleInit+45));

//    q1 = (-d)/(2*kt*(d2+d));
//    q2 = (1)/(2*d1*kt);
//    q3 = (1)/(2*(d2*kt+d*kt));
//    rpm_motor1 = ((q1*throttle_input)*-1) - (q2*ControlRoll) + (q3*ControlYaw);
//    rpm_motor2 = ((q1*throttle_input)*-1) + (q2*ControlRoll) + (q3*ControlYaw);
//    motor1 =  throttle_input -  ControlRoll +  ControlPitch;
//		motor2 =  throttle_input +  ControlRoll +  ControlPitch;
//    motor3 =  sqrt(throttle_input*throttle_input+ throttle_input * ControlPitch + ControlPitch*ControlPitch - throttle_input*ControlYaw + ControlYaw*ControlYaw);
//    motor3 = sqrt((d2*d2*throttle_input*throttle_input*d*d*kd*kd+2*d2*throttle_input*ControlPitch*d*d*kd*kd+ControlPitch*ControlPitch*d*d*kd*kd+throttle_input*throttle_input*kd*kd*(d2+d)*(d2+d)-2*throttle_input*ControlYaw*kt*kd*(d2+d)*(d2+d)+ControlYaw*ControlYaw*kt*kt*(d2+d)*(d2+d))/(d*d*kt*kt*kd*kd*(d2+d)*(d2+d)));
//		servo  = acos(sqrt((d2*d2*throttle_input*throttle_input+2*d2*throttle_input*ControlPitch+ControlPitch*ControlPitch)/(kt*kt*(d2+d)*(d2*d))));


    if (motor1 < 1065){ motor1 = 1065; }
		if (motor2 < 1065){ motor2 = 1065; }
		if (motor3 < 1065){ motor3 = 1065; }

		if (motor1 > 1700){ motor1 = 1700; }
		if (motor2 > 1700){ motor2 = 1700; }
		if (motor3 > 1700){ motor3 = 1700; }
  }

  motor_update();
  update_servo();
}
