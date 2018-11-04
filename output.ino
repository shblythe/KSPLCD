void output() {
  now = millis();
  controlTime = now - controlTimeOld;
  if (controlTime > CONTROLREFRESH) {
    controlTimeOld = now;
    controls();
  }
}

/*
 * Notes: this autopilot is currently designed to drive Kerbal 1 only
 * TODO
 * D Add state to move to P45, R0, H90 attitude, during 2nd stage?
 * D Add state to move to P90, R0, H90 attitude, (or just SAS prograde?) during 1st stage?
 * D Make SAS target retrograde on re-entry
 * - Split into programs:
 *  D P01 - Launch, roll to 45 deg and burn until APO 80000
 *  * P02 - Circularise Orbit at 80000
 *    - improve with more accurate calculations of max burn time
 *      - use remaining fuel mass
 *  * P03 - Reentry burn, reentry and chute
 *    - retest since radar alt change
 */
void AutoPilot_P01()
{
  static int state=0;
  static long state_inc_time=0;
  static long wait_time;
  static int attitude_state=0;
  static int attitude_count=0;
  char line1[17];
  char line2[17];
  char buf1[17];
  char buf2[17];
  char buf3[17];
  line2[0]='\0';

  if (state_inc_time!=0)
  {
    if (millis()>state_inc_time)
    {
      state_inc_time=0;
      state++;
    }
  }
  else
  {
    switch (state)
    {
      case 0:
        // Prepare to stage.  Start 5s countdown.
        MainControls(STAGE,LOW);
        wait_time=millis()+5000;
        state_inc_time=millis()+1;
        break;
      case 1:
        // 5s countdown
        if (millis()>wait_time)
          state_inc_time=millis()+1;
        snprintf(line2,17,"COUNTDOWN: %d",wait_time-millis());
        break;
      case 2:
        // Max throttle
        CPacket.Throttle=1000;
        state_inc_time=millis()+100;
        break;
      case 3:
        // SAS on
        MainControls(SAS,HIGH);
        setSASMode(SMSAS);
        state_inc_time=millis()+100;
        break;
      case 4:
        // Stage 5 - solid boosters and main engine
        MainControls(STAGE,HIGH);
        state_inc_time=millis()+1000;
        break;
      case 5:
        // Stage 5 complete
        MainControls(STAGE,LOW);
        if (VData.SolidFuel<0.01)
          state_inc_time=millis()+100;
        //dispValue(VData.SolidFuel,2,buf1,10,false,false);
        //snprintf(line2,17,"FUEL: %10s\n",buf1);
        break;
      case 6:
        // Stage 4 - detach boosters
        MainControls(STAGE,HIGH);
        state_inc_time=millis()+1000;
        attitude_state=0;
        break;
      case 7:
        // Stage 4 complete
        // Pitch gently to 45 degrees
        MainControls(STAGE,LOW);
        if (VData.LiquidFuelS<0.01 || VData.AP>=80000)
          state_inc_time=millis()+1;
        //snprintf(line2,17,"LF:%4d AP:%5d",VData.LiquidFuelS,VData.AP);
        //dispValue(VData.LiquidFuelS,2,buf1,10,false,false);
        //snprintf(line2,17,"FUEL: %10s\n",buf1);
        if (VData.Pitch>=45)
        {
          CPacket.Yaw=500;
          CPacket.Roll=0;
          CPacket.Pitch=0;
          attitude_count=0;
        }
        else
        {
          CPacket.Yaw=0;
          CPacket.Roll=0;
          CPacket.Pitch=0;
          attitude_count=0;
        }
        break;
      case 8:
        // Main engine fuel exhausted or target apoapsis reached
        CPacket.Yaw=0;
        CPacket.Roll=0;
        CPacket.Pitch=0;
        if (VData.AP>=80000)
          CPacket.Throttle=0;
        state_inc_time=millis()+100;
        break;
      case 9:
        // Stage 3 - detach main engine
        MainControls(STAGE,HIGH);
        state_inc_time=millis()+1000;
        break;
      case 10:
        // Stage 3 complete
        MainControls(STAGE,LOW);
        state_inc_time=millis()+1000;
        break;
      case 11:
        // Stage 2 - start orbital engine
        MainControls(STAGE,HIGH);
        state_inc_time=millis()+1000;
        break;
      case 12:
        // Stage 2 complete
        // Target pitch 15 degrees
        MainControls(STAGE,LOW);
        if (VData.LiquidFuelS<0.01 || VData.AP>=80000)
          state_inc_time=millis()+1000;
        if (VData.AP>=80000)
          CPacket.Throttle=0;
        if (VData.Pitch>=45)
        {
          CPacket.Yaw=500;
          CPacket.Roll=0;
          CPacket.Pitch=0;
          attitude_count=0;
        }
        else
        {
          CPacket.Yaw=0;
          CPacket.Roll=0;
          CPacket.Pitch=0;
          attitude_count=0;
        }
        break;
      case 13:
        state=0;
        mode=0;
    }
  }
  /*
  else
    snprintf(line2,17," AWAITING  DATA ");
  */
  if (line2[0]=='\0')
  {
    dispValue(VData.Pitch,0,buf1,3,false,false);
    dispValue(VData.Roll,0,buf2,3,false,false);
    dispValue(VData.Heading,0,buf3,3,true,false);
    snprintf(line2,17,"P%4s R%4s H%3s",buf1,buf2,buf3);
  }
  lcd.clear();
  snprintf(line1,17,"STATE%2d ST%d/%d",state,VData.CurrentStage,VData.TotalStage);
  lcd.print(line1);
  lcd.setCursor(0,1);
  lcd.print(line2);
  
}

void AutoPilot_P02()
{
  static int state=0;
  static long state_inc_time=0;
  static long wait_time;
  static int attitude_state=0;
  static int attitude_count=0;
  static float max_dt_burn;
  const float mu=3.5316e12; // Kerbin gravitational parameter 3.5316x10^12 m^3/s^2
  const float body_radius=600000.0; // Kerbin 600000m
  float r_pe;  // Have to add radius of Kerbin, as this is from centre not surface
  float r_ap;
  float dv_burn;
  const float mass=2500;  // K1 when staged down to LV-909, approx 3t - WITHOUT FUEL
  const float fuel_mass=2500;
  const float force=60000;  // LV-909, 60kN @ Vac.
  char line1[17];
  char line2[17];
  char buf1[17];
  char buf2[17];
  char buf3[17];
  float diff;
  line2[0]='\0';

  if (state_inc_time!=0)
  {
    if (millis()>state_inc_time)
    {
      state_inc_time=0;
      state++;
    }
  }
  else
  {
    switch (state)
    {
      case 0:
        // Switch on SAS, and set to prograde, then wait 3s
        MainControls(SAS, HIGH);
        setSASMode(SMPrograde); //setting SAS mode
        state_inc_time=millis()+3000;
        // Calculations from https://www.reddit.com/r/KerbalAcademy/comments/1zn3fu/how_much_deltav_do_i_need_for_circularizing_an/
        // and https://www.reddit.com/r/KerbalAcademy/comments/1oremg/q_is_there_a_way_to_manually_calculate_burn_time/
        r_pe=VData.PE+body_radius;  // Have to add radius of Kerbin, as this is from centre not surface
        r_ap=VData.AP+body_radius;
        dv_burn=sqrt(mu/r_ap)-sqrt((r_pe*mu)/(r_ap*(r_pe+r_ap)/2));
        max_dt_burn=dv_burn*(mass+VData.LiquidFuelS/VData.LiquidFuelTotS*fuel_mass)/force; // burn length in seconds
        break;
      case 1:
        // Wait until time to apoapsis < half max burn time
        dispValue(max_dt_burn,0,buf1,7,false,false);
        dispValue(VData.TAp,0,buf2,7,false,false);
        snprintf(line2,17,"%7s %7s",buf1,buf2);
        if (VData.TAp<max_dt_burn/2 || VData.TAp>15*60) // Burn if we're within, or we've overshot
        {
          CPacket.Throttle=1000;
          state_inc_time=millis()+1;
        }
        break;
      case 2:
        // Now control burn according to TAp, turning off when >30 and on when <10
        diff=VData.AP-VData.PE;
        if (VData.TAp>max_dt_burn/2 && VData.TAp<15*60)
        {
          CPacket.Throttle=0;
          state=0;
        }
        else if (diff<1000)
        {
          CPacket.Throttle=0;
          state_inc_time=millis()+1;
        }
        else
        {
          // Set throttle according to how much burn left to go
          if (diff>20000)
            CPacket.Throttle=1000;
          else if (diff>10000)
            CPacket.Throttle=500;
          else if (diff>5000)
            CPacket.Throttle=200;
          else
            CPacket.Throttle=100;
        }
        break;
      case 3:
        state=0;
        mode=0;
    }
  }
  /*
  else
    snprintf(line2,17," AWAITING  DATA ");
  if (line2[0]=='\0')
  {
    dispValue(VData.Pitch,0,buf1,3,false,false);
    dispValue(VData.Roll,0,buf2,3,false,false);
    dispValue(VData.Heading,0,buf3,3,true,false);
    snprintf(line2,17,"P%4s R%4s H%3s",buf1,buf2,buf3);
  }
  */
  lcd.clear();
  snprintf(line1,17,"STATE%2d ST%d/%d",state,VData.CurrentStage,VData.TotalStage);
  lcd.print(line1);
  lcd.setCursor(0,1);
  lcd.print(line2);
  
}

void AutoPilot_P03()
{
  static int state=0;
  static long state_inc_time=0;
  static long wait_time;
  static int attitude_state=0;
  static int attitude_count=0;
  char line1[17];
  char line2[17];
  char buf1[17];
  char buf2[17];
  char buf3[17];
  line2[0]='\0';

  if (state_inc_time!=0)
  {
    if (millis()>state_inc_time)
    {
      state_inc_time=0;
      state++;
    }
  }
  else
  {
    switch (state)
    {
      case 0:
        // Switch on SAS, and set to retrograde, then wait 3s
        MainControls(SAS, HIGH);
        setSASMode(SMRetroGrade); //setting SAS mode
        state_inc_time=millis()+3000;
        break;
      case 1:
        // Burn 50% until periapsis <40k
        CPacket.Throttle=500;
        state_inc_time=millis()+1;
        break;
      case 2:
        if (VData.PE<40000)
        {
          CPacket.Throttle=0;
          state_inc_time=millis()+1;
        }
        break;
      case 3:
        MainControls(STAGE,HIGH);   // jettison engine
        state_inc_time=millis()+1000;
        break;
      case 4:
        MainControls(STAGE,LOW);
        state_inc_time=millis()+1000;
        break;
      case 5:
        if (VData.RAlt<7000)  // Use radar altitude for chute deploy in case we're over mountains
        {
          MainControls(STAGE,HIGH);   // deploy chute
          state_inc_time=millis()+1000;
        }
        break;
      case 6:
        MainControls(STAGE,LOW);
        state=0;
        mode=0;
    }
  }
  /*
  else
    snprintf(line2,17," AWAITING  DATA ");
  */
  if (line2[0]=='\0')
  {
    dispValue(VData.Pitch,0,buf1,3,false,false);
    dispValue(VData.Roll,0,buf2,3,false,false);
    dispValue(VData.Heading,0,buf3,3,true,false);
    snprintf(line2,17,"P%4s R%4s H%3s",buf1,buf2,buf3);
  }
  lcd.clear();
  snprintf(line1,17,"STATE%2d ST%d/%d",state,VData.CurrentStage,VData.TotalStage);
  lcd.print(line1);
  lcd.setCursor(0,1);
  lcd.print(line2);
  
}

void AutoPilot_P04()
{
  static int state=0;
  static long state_inc_time=0;
  static long wait_time;
  static int attitude_state=0;
  static int attitude_count=0;
  static float max_dt_burn;
  const float mass=2500;  // K1 when staged down to LV-909, approx 3t - WITHOUT FUEL
  const float fuel_mass=2500;
  const float force=60000;  // LV-909, 60kN @ Vac.
  char line1[17];
  char line2[17];
  char buf1[17];
  char buf2[17];
  char buf3[17];
  int32_t time_to_mn=VData.MNTime;
  line2[0]='\0';

  if (state_inc_time!=0)
  {
    if (millis()>state_inc_time)
    {
      state_inc_time=0;
      state++;
    }
  }
  else
  {
    switch (state)
    {
      case 0:
        // Switch on SAS, and set to manouevre node, then wait 3s
        MainControls(SAS, HIGH);
        setSASMode(SMManeuverNode); //setting SAS mode
        state_inc_time=millis()+3000;
        // Calculations from https://www.reddit.com/r/KerbalAcademy/comments/1oremg/q_is_there_a_way_to_manually_calculate_burn_time/
        max_dt_burn=VData.MNDeltaV*(mass+VData.LiquidFuelS/VData.LiquidFuelTotS*fuel_mass)/force; // burn length in seconds
        break;
      case 1:
        // Wait until time to manouevre < half max burn time
        dispValue(max_dt_burn,0,buf1,7,false,false);
        dispValue(time_to_mn,0,buf2,7,false,false);
        snprintf(line2,17,"%7s %7s",buf1,buf2);
        if (time_to_mn<max_dt_burn/2) // Burn if we're within, or we've overshot
        {
          CPacket.Throttle=1000;
          state_inc_time=millis()+1;
        }
        break;
      case 2:
        // Now control burn according to TAp, turning off when >30 and on when <10
        if (time_to_mn>max_dt_burn/2)
        {
          CPacket.Throttle=0;
          state=0;
        }
        else if (VData.MNDeltaV<0.5f)
        {
          CPacket.Throttle=0;
          state_inc_time=millis()+1;
        }
        else
        {
          // Set throttle according to how much burn left to go
          if (VData.MNDeltaV>50)
            CPacket.Throttle=1000;
          else if (VData.MNDeltaV>20)
            CPacket.Throttle=500;
          else if (VData.MNDeltaV>5)
            CPacket.Throttle=200;
          else if (VData.MNDeltaV>1)
            CPacket.Throttle=100;
          else
            CPacket.Throttle=50;
        }
        break;
      case 3:
        state=0;
        mode=0;
    }
  }
  /*
  else
    snprintf(line2,17," AWAITING  DATA ");
  if (line2[0]=='\0')
  {
    dispValue(VData.Pitch,0,buf1,3,false,false);
    dispValue(VData.Roll,0,buf2,3,false,false);
    dispValue(VData.Heading,0,buf3,3,true,false);
    snprintf(line2,17,"P%4s R%4s H%3s",buf1,buf2,buf3);
  }
  */
  lcd.clear();
  snprintf(line1,17,"STATE%2d ST%d/%d",state,VData.CurrentStage,VData.TotalStage);
  lcd.print(line1);
  lcd.setCursor(0,1);
  lcd.print(line2);
  
}

void AutoPilot()
{
  static int state=0;
  static long state_inc_time=0;
  static long wait_time;
  static int attitude_state=0;
  static int attitude_count=0;
  char line1[17];
  char line2[17];
  char buf1[17];
  char buf2[17];
  char buf3[17];
  line2[0]='\0';

  if (state_inc_time!=0)
  {
    if (millis()>state_inc_time)
    {
      state_inc_time=0;
      state++;
    }
  }
  else
  {
    switch (state)
    {
      case 0:
        // Prepare to stage.  Start 5s countdown.
        MainControls(STAGE,LOW);
        wait_time=millis()+5000;
        state_inc_time=millis()+1;
        break;
      case 1:
        // 5s countdown
        if (millis()>wait_time)
          state_inc_time=millis()+1;
        snprintf(line2,17,"COUNTDOWN: %d",wait_time-millis());
        break;
      case 2:
        // Max throttle
        CPacket.Throttle=1000;
        state_inc_time=millis()+100;
        break;
      case 3:
        // SAS on
        MainControls(SAS,HIGH);
        setSASMode(SMSAS);
        state_inc_time=millis()+100;
        break;
      case 4:
        // Stage 5 - solid boosters and main engine
        MainControls(STAGE,HIGH);
        state_inc_time=millis()+1000;
        break;
      case 5:
        // Stage 5 complete
        MainControls(STAGE,LOW);
        if (VData.SolidFuel<0.01)
          state_inc_time=millis()+100;
        //dispValue(VData.SolidFuel,2,buf1,10,false,false);
        //snprintf(line2,17,"FUEL: %10s\n",buf1);
        break;
      case 6:
        // Stage 4 - detach boosters
        MainControls(STAGE,HIGH);
        state_inc_time=millis()+1000;
        attitude_state=0;
        break;
      case 7:
        // Stage 4 complete
        // Pitch gently to 45 degrees
        MainControls(STAGE,LOW);
        if (VData.LiquidFuelS<0.01 || VData.AP>=80000)
          state_inc_time=millis()+1000;
        //snprintf(line2,17,"LF:%4d AP:%5d",VData.LiquidFuelS,VData.AP);
        //dispValue(VData.LiquidFuelS,2,buf1,10,false,false);
        //snprintf(line2,17,"FUEL: %10s\n",buf1);
        if (VData.Pitch>=45)
        {
          CPacket.Yaw=500;
          CPacket.Roll=0;
          CPacket.Pitch=0;
          attitude_count=0;
        }
        else
        {
          CPacket.Yaw=0;
          CPacket.Roll=0;
          CPacket.Pitch=0;
          attitude_count=0;
        }
        break;
      case 8:
        // Main engine fuel exhausted or target apoapsis reached
        CPacket.Yaw=0;
        CPacket.Roll=0;
        CPacket.Pitch=0;
        if (VData.AP>=80000)
          CPacket.Throttle=0;
        state_inc_time=millis()+100;
        break;
      case 9:
        // Stage 3 - detach main engine
        MainControls(STAGE,HIGH);
        state_inc_time=millis()+1000;
        break;
      case 10:
        // Stage 3 complete
        MainControls(STAGE,LOW);
        state_inc_time=millis()+1000;
        break;
      case 11:
        // Stage 2 - start orbital engine
        MainControls(STAGE,HIGH);
        state_inc_time=millis()+1000;
        break;
      case 12:
        // Stage 2 complete
        // Target pitch 15 degrees
        MainControls(STAGE,LOW);
        if (VData.LiquidFuelS<0.01 || VData.AP>=80000)
          state_inc_time=millis()+1000;
        //snprintf(line2,17,"LF:%4d AP:%5d",VData.LiquidFuelS,VData.AP);
        //dispValue(VData.LiquidFuelS,2,buf1,10,false,false);
        //snprintf(line2,17,"FUEL: %10s\n",buf1);
        if (VData.Pitch>20)
        {
          CPacket.Yaw=100;
          CPacket.Roll=0;
          CPacket.Pitch=0;
        }
        else if (VData.Pitch<10)
        {
          CPacket.Yaw=-100;
          CPacket.Roll=0;
          CPacket.Pitch=0;
        }
        else
        {
          CPacket.Yaw=0;
          CPacket.Roll=0;
          CPacket.Pitch=0;
        }
        break;
      case 13:
        // Orbital engine fuel exhausted or target apoapsis reached
        if (VData.AP>=80000)
          CPacket.Throttle=0;
        state_inc_time=millis()+1000;
        break;
      case 14:
        // Stage 1 - detach orbital engine
        MainControls(STAGE,HIGH);
        state_inc_time=millis()+1000;
        break;
      case 15:
        // Stage 1 complete
        // Re-entry
        MainControls(STAGE,LOW);
        setSASMode(SMRetroGrade);
        if (VData.Alt<15000)
          state_inc_time=millis()+1000;
        break;
      case 16:
        // Stage 0 - parachute deploy
        MainControls(STAGE,HIGH);
        state=0;
        mode=0;
        break;
    }
  }
  /*
  else
    snprintf(line2,17," AWAITING  DATA ");
  */
  if (line2[0]=='\0')
  {
    dispValue(VData.Pitch,0,buf1,3,false,false);
    dispValue(VData.Roll,0,buf2,3,false,false);
    dispValue(VData.Heading,0,buf3,3,true,false);
    snprintf(line2,17,"P%4s R%4s H%3s",buf1,buf2,buf3);
  }
  lcd.clear();
  snprintf(line1,17,"STATE%2d ST%d/%d",state,VData.CurrentStage,VData.TotalStage);
  lcd.print(line1);
  lcd.setCursor(0,1);
  lcd.print(line2);
}

void controls() {
  if (Connected) {

    if (!digitalRead(MODEPIN))
    {
      if (mode_pressed==false)
      {
        mode++;
        if (mode==MODE_NORM_MAX+1)
          mode=MODE_AUTO_READY;
        else if (mode>MODE_AUTO_MAX)
          mode=0;
        mode_pressed=true;
      }
    }
    else
    {
      mode_pressed=false;
    }
    
    if (mode==MODE_AUTO)
      AutoPilot();
    else if (mode==MODE_AUTO_P01)
      AutoPilot_P01();
    else if (mode==MODE_AUTO_P02)
      AutoPilot_P02();
    else if (mode==MODE_AUTO_P03)
      AutoPilot_P03();
    else if (mode==MODE_AUTO_P04)
      AutoPilot_P04();
    else if (mode>=MODE_AUTO_READY && !digitalRead(AUTOPIN))
    {
      mode+=MODE_AUTO_OFFSET;
    }
    else
    {
      if (digitalRead(SASPIN)) { //--------- This is how you do main controls
        MainControls(SAS, HIGH);
        setSASMode(SMSAS); //setting SAS mode
        //setNavballMode(NAVBallSURFACE); //setting navball mode
      }
      else {
        //setNavballMode(NAVBallTARGET);
        MainControls(SAS, LOW);
      }
  
  /*
      if (digitalRead(RCSPIN))
        MainControls(RCS, HIGH);
      else
        MainControls(RCS, LOW);
      if (digitalRead(CG1PIN))   //--------- This is how you do control groups
        ControlGroups(1, HIGH);
      else
        ControlGroups(1, LOW);
  */      
      /*
         if (getSASMode() == SMPrograde) { //--------- This is how you read SAS modes
           //Blink LED, do stuff, etc.
         }
  
         if (getNavballMode() == NAVBallTARGET) { //--------- This is how you read navball modes
           //Blink LED, do stuff, etc.
         }
      */
  
  
      //This is an example of reading analog inputs to an axis, with deadband and limits
      CPacket.Throttle = constrain(map(analogRead(THROTTLEPIN), THROTTLEDB, 1024 - THROTTLEDB, 0, 1000), 0, 1000);
  
      //This is an example of reading analog inputs to an axis, with deadband and limits
      //CPacket.Pitch = constrain(map(analogRead(THROTTLEPIN),0,1024,-1000,1000),-1000, 1000);
    }
    KSPBoardSendData(details(CPacket));
  }
}

void controlsInit() {
  pinMode(SASPIN, INPUT_PULLUP);
  //pinMode(RCSPIN, INPUT_PULLUP);
  pinMode(MODEPIN, INPUT_PULLUP);
  pinMode(AUTOPIN, INPUT_PULLUP);
}

byte getSASMode() {
  return VData.NavballSASMode & B00001111; // leaves alone the lower 4 bits of; all higher bits set to 0.
}

byte getNavballMode() {
  return VData.NavballSASMode >> 4; // leaves alone the higher 4 bits of; all lower bits set to 0.
}

void setSASMode(byte m) {
  CPacket.NavballSASMode &= B11110000;
  CPacket.NavballSASMode += m;
}

void setNavballMode(byte m) {
  CPacket.NavballSASMode &= B00001111;
  CPacket.NavballSASMode += m << 4;
}

void MainControls(byte n, boolean s) {
  if (s)
    CPacket.MainControls |= (1 << n);       // forces nth bit of x to be 1.  all other bits left alone.
  else
    CPacket.MainControls &= ~(1 << n);      // forces nth bit of x to be 0.  all other bits left alone.
}

void ControlGroups(byte n, boolean s) {
  if (s)
    CPacket.ControlGroup |= (1 << n);       // forces nth bit of x to be 1.  all other bits left alone.
  else
    CPacket.ControlGroup &= ~(1 << n);      // forces nth bit of x to be 0.  all other bits left alone.
}


