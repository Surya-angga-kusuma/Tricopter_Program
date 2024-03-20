void servo_setup()
{
   myservo.write(servoAngleInit);
}

void update_servo()
{
  timeServo = micros();
  if(timeServo - previousTimeServo >= 10000)
  {
   myservo.write(servo);
   previousTimeServo = micros();
  }
}

void motor_setup()
{
  pinMode(MOTOR1, OUTPUT);
  pinMode(MOTOR2, OUTPUT);
  pinMode(MOTOR3, OUTPUT);

  brushless1.attach(MOTOR1);
  brushless2.attach(MOTOR2);
  brushless3.attach(MOTOR3);

  brushless1.writeMicroseconds(1000);
  brushless2.writeMicroseconds(1000);
  brushless3.writeMicroseconds(1000);

  motor1 = 1000;
  motor2 = 1000;
  motor3 = 1000;
  delay(1000);
}

void motor_update()
{
    brushless1.writeMicroseconds(motor1);
    brushless2.writeMicroseconds(motor2);
    brushless3.writeMicroseconds(motor3);
}
