
int input() {
  int returnValue = -1;
  now = millis();

  if (KSPBoardReceiveData()){
    deadtimeOld = now;
    returnValue = id;
    switch(id) {
    case 0: //Handshake packet
      Handshake();
      break;
    case 1:
      Indicators();
      break;
    }

    //We got some data, turn the green led on
    shifter->setBit(GLED);
    Connected = true;
  }
  else
  { //if no message received for a while, go idle
    deadtime = now - deadtimeOld; 
    if (deadtime > IDLETIMER)
    {
      //lcd.clear();
      //lcd.print("-- LINK  LOST --");
      deadtimeOld = now;
      Connected = false;
      LEDSAllOff();
    }    
  }

  return returnValue;
}

byte ControlStatus(byte n)
{
  return ((VData.ActionGroups >> n) & 1) == 1;
}

