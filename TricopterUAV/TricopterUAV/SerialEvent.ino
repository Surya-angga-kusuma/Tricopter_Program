void serialEvent() 
{
  while (Serial.available())
  { 
    inChar = Serial.read();
//    if (inChar == 'a'){ servoAngleInit += 1;}
//    if (inChar == 's'){ servoAngleInit -= 1;}

    if (inChar == 'w'){ GainProll += 0.1;}
    if (inChar == 'q'){ GainProll -= 0.1;}
    if (inChar == 's'){ GainDroll += 0.01;}
    if (inChar == 'a'){ GainDroll -= 0.01;}
    if (inChar == 'x'){ GainIroll += 0.01;}
    if (inChar == 'z'){ GainIroll -= 0.01;}

//    if (inChar == 'a'){ GainPpitch += 0.1;}
//    if (inChar == 's'){ GainPpitch -= 0.1;}
//    if (inChar == 'w'){ GainDpitch += 0.1;}
//    if (inChar == 'q'){ GainDpitch -= 0.1;}
//    if (inChar == 'x'){ GainIpitch += 0.1;}
//    if (inChar == 'z'){ GainIpitch -= 0.1;}

//    if (inChar == 'a'){ GainPyaw += 0.1;}
//    if (inChar == 's'){ GainPyaw -= 0.1;}
//    if (inChar == 'w'){ GainPyaw += 0.1;}
//    if (inChar == 'q'){ GainPyaw -= 0.1;}
//    if (inChar == 'x'){ GainPyaw += 0.1;}
//    if (inChar == 'z'){ GainPyaw -= 0.1;}

   }
}
