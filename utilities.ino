#include <stdio.h>
#include <math.h>

const char mult_chars[]="afpnum kMGTPE";
const int default_mult=6;

//convert a value to an integer with the specified number of digits, decimal places, and a multiplier, e.g. M, k etc.
//value - the value to convert
//dp - the number of decimal places
//str - the destination string
//len - the total length of the string, excluding '\0'
void dispValue(float value, int dp, char *str, int len, bool dashneg, bool scale)
{
  int numlen=len;
  if (scale)
    numlen-=1; // subtract 1 for the multiplier
  if (value<0 && dashneg)
  {
    for (int i=0; i<len; i++)
      str[i]='-';
    str[len]='\0';
    return;
  }
  else
    numlen=len-1; // subtract 1 for the sign
  int mult=default_mult;  // default multiplier is none (1)
  if (dp>0)
    numlen-=(dp+1);  // subtract the number of DPs, plus the space for the DP itself
  int maxnum=pow(10,numlen);
  float minnum=powf(10,-dp);
  if (scale)
  {
    while (fabsf(value)>maxnum)
    {
      value/=1000.0;
      mult+=1;
    }
    while (value!=0.0 && fabsf(value)<minnum)
    {
      value*=1000.0;
      mult-=1;
    }
  }
  if (dp>0)
  {
    dtostrf(value,numlen+dp+1,dp,str);
    if (scale)
      str[numlen+dp+1]=mult_chars[mult];
  }
  else    
  {
    char fmt[80];
    if (scale)
    {
      snprintf(fmt,80,"%%%dd%%c",numlen);
      snprintf(str,len+1,fmt,(int)value,mult_chars[mult]);
    }
    else
    {
      snprintf(fmt,80,"%%%dd",numlen);
      snprintf(str,len+1,fmt,(int)value);
    }
  }
}

void dispTime(int value, char *str, int len)
{
  snprintf(str,len+1,"%02d:%02d",value/60,value%60);
}


void LCDIndicators() 
{
  char line1[17];
  char line2[17];
  int ap_num,pe_num;
  if (mode==MODE_LAUNCH)  // LAUNCH
  {
    char vvi_string[6],g_string[4],den_string[5],stg_fuel_string[4];
    dispValue(VData.VVI,0,vvi_string,5,false,false);
    dispValue(VData.G,1,g_string,3,true,false);
    dispValue(VData.Density,3,den_string,5,true,false);
    dispValue(VData.LiquidFuelS/VData.LiquidFuelTotS*100,0,stg_fuel_string,3,true,false);
    snprintf(line1,17,"VVI%5s G%3s    ",vvi_string,g_string);
    snprintf(line2,17,"DN%5s S%2s%%",den_string,stg_fuel_string);
  }
  else if (mode==MODE_ORBIT) // ORBIT
  {
    char ap_string[5],pe_string[5];
    char tap_string[6],tpe_string[6];
    dispValue(VData.AP,0,ap_string,4,true,true);
    dispValue(VData.PE,0,pe_string,4,true,true);
    dispTime(VData.TAp,tap_string,5);
    dispTime(VData.TPe,tpe_string,5);
    snprintf(line1,17,"A%4s:%5s @%3d",ap_string,tap_string,VData.SOINumber);
    snprintf(line2,17,"P%4s:%5s   ",pe_string,tpe_string);
  }
  else
  {
    snprintf(line1,17," AUTOPILOT  P%02d ",mode-MODE_AUTO_READY);
    snprintf(line2,17,"     ENGAGE     ");
  }
  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print(line1);
  lcd.setCursor(0,1);
  lcd.print(line2);
  
  lcd.setCursor(13,1);
  snprintf(line2,17," M%d",mode);
  lcd.print(line2);
}


void Indicators() {
  caution = 0;
  warning = 0;

  caution += VData.G > GCAUTION;
  warning += VData.G > GWARN;
  caution += VData.LiquidFuelS/VData.LiquidFuelTotS*100 < FUELCAUTION;
  warning += VData.LiquidFuelS/VData.LiquidFuelTotS*100 < FUELWARN;

  if (caution != 0)
    shifter->setBit(YLED);
  else
    shifter->clearBit(YLED);

  if (warning != 0)
    shifter->setBit(RLED);
  else
    shifter->clearBit(RLED);

  if (ControlStatus(AGSAS))
    shifter->setBit(SASLED);
  else
    shifter->clearBit(SASLED);

  if (ControlStatus(AGRCS))
    shifter->setBit(RCSLED);
  else
    shifter->clearBit(RCSLED);
  if (mode!=99)
    LCDIndicators();
}

void initLEDS() {
  shifter->allOn();
}

void LEDSAllOff() {
  shifter->allOff();
}


void InitTxPackets() {
  HPacket.id = 0;  
  CPacket.id = 101;
}








