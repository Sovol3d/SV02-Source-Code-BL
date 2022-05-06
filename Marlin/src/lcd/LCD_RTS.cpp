#include "LCD_RTS.h"
#include <HardwareSerial.h>
#include <arduino.h>
#include <wstring.h>
#include <stdio.h>
#include "../inc/MarlinConfig.h"

#include "../Marlin.h"
#include "../core/language.h"
#include "../sd/cardreader.h"
#include "../module/temperature.h"
#include "../module/planner.h"
#include "../module/stepper.h"
#include "../module/configuration_store.h"
#include "../module/motion.h"
#include "../module/printcounter.h"
#include "../libs/duration_t.h"
#include "../feature/babystep.h"
#include "../feature/bedlevel/bedlevel.h"
#include "../feature/power_loss_recovery.h"
#include "../gcode/parser.h"
#include "../gcode/gcode.h"
#include "../gcode/queue.h"
#include "ultralcd.h"

#if ENABLED(BLTOUCH)
  #include "../module/endstops.h"
#endif

#define CHECKFILEMENT true

#if ENABLED(BABYSTEPPING)
  float last_zoffset ;
#endif

int power_off_type_yes;
int power_off_commands_count;
int startprogress = 0;
CRec CardRecbuf;
int temphot = 0;
int tempbed = 0;
float pause_z = 0;
float pause_e = 0;

float PLA_ABSModeTemp = 195;
millis_t next_rts_update_ms = 0;
int last_temp_bed;
int last_target_temperature[4] = {0};
char waitway = 0;
int recnum = 0;
unsigned char original_extruder = 0;
unsigned char Percentrecord = 0;
float ChangeMaterialbuf[2] = {0};
char NozzleTempStatus[3] = {0};

int PrintModeTime = 0;
bool PrintModeChange = true;
// PrintStatue[0], 0 represent  to 43 page, 1 represent to 44 page
char PrintStatue[2] = {0};

// represents to update file list
bool CardUpdate = false;

// CardCheckStatus[0] represents to check card in printing and after making sure to begin and to check card in heating with value as 1, but 0 for off
char CardCheckStatus[2] = {0};

// !0 represent Chinese, 0 represent English
unsigned char LanguageRecbuf;

// PrinterStatusKey[1] value: 0 represents to keep temperature, 1 represents  to heating , 2 stands for cooling , 3 stands for printing
// PrinterStatusKey[0] value: 0 reprensents 3D printer ready
char PrinterStatusKey[2] = {0};
bool PreheatStatus[] = {false,false};
unsigned char Average_count = 0;
bool Temp_conditions = false;

extern CardReader card;
// represents SD-card status, true means SD is available, false means opposite.
bool LCD_sd_status;

char Checkfilenum = 0;
char FilenamesCount = 0;
char cmdbuf[20] = {0};
char FilementStatus[2] = {0};

// 0 for 10mm, 1 for 1mm, 2 for 0.1mm
unsigned char AxisPagenum = 0;
bool InforShowStatus = true;

// true for only opening time and percentage, false for closing time and percentage.
bool TPShowStatus = false;
bool FanStatus = true;
bool AutohomeKey = false;
bool AutoLevelStatus = false;
unsigned char AutoHomeIconNum;
RTSSHOW rtscheck;
int Update_Time_Value = 0;
unsigned long VolumeSet = 0x80;

bool PoweroffContinue = false;
char commandbuf[30];
unsigned char G29_status = 0;
int8_t showcount;

inline void RTS_line_to_current(AxisEnum axis) 
{
  if (!planner.is_full())
  {
	const feedRate_t fr_mm_s = MMM_TO_MMS(manual_feedrate_mm_m[axis]);
	planner.buffer_line(current_position, fr_mm_s, active_extruder);
  }
}

void Move_Z(const float &lz, unsigned int multiple=0)
{
  do_blocking_move_to_z(lz,multiple * MMM_TO_MMS(HOMING_FEEDRATE_Z));
}

void Move_XY(const float &lx, const float &ly, unsigned int multiple=0)
{
  do_blocking_move_to_xy(lx,ly ,multiple *MMM_TO_MMS(HOMING_FEEDRATE_XY));
}

RTSSHOW::RTSSHOW()
{
  recdat.head[0] = snddat.head[0] = FHONE;
  recdat.head[1] = snddat.head[1] = FHTWO;
  memset(databuf,0, sizeof(databuf));
}

void RTSSHOW::RTS_SDCardInit(void)
{
  if(!card.isMounted())
  {
    card.mount(); 
  }
  delay(2);
  if (card.isMounted())
  {
    SERIAL_ECHOLN("***Initing card is OK***");
    uint16_t fileCnt = card.countFilesInWorkDir();
    card.getWorkDirName();
    if (card.filename[0] == '/') 
    {
      card.mount();
    }
    else 
    {
      card.cdup();
    }

    int addrnum =0;
    int num = 0;
    for (uint16_t i = 0; i < fileCnt && i < MaxFileNumber + addrnum; i++) 
    {
      card.selectFileByIndex(fileCnt-1-i);
      char *pointFilename = card.longFilename;
      int filenamelen = strlen(card.longFilename);
      int j = 1;
      while((strncmp(&pointFilename[j],".gcode",6) && strncmp(&pointFilename[j],".GCODE",6)) && (j++) < filenamelen);
      if(j >= filenamelen)
      {
        addrnum++;
        continue;
      }

      if(j >= TEXTBYTELEN)	
      {
        strncpy(&card.longFilename[TEXTBYTELEN -3],"~~",2);
        card.longFilename[TEXTBYTELEN-1] = '\0';
        j = TEXTBYTELEN-1;
      }

      delay(3);
      strncpy(CardRecbuf.Cardshowfilename[num], card.longFilename,j);

      strcpy(CardRecbuf.Cardfilename[num], card.filename);
      CardRecbuf.addr[num] = SDFILE_ADDR +num*10;
      RTS_SndData(CardRecbuf.Cardshowfilename[num],CardRecbuf.addr[num]);
      CardRecbuf.Filesum = (++num);
      RTS_SndData(1, FilenameIcon + CardRecbuf.Filesum);
    }
    if(LanguageRecbuf != 0)
    {
      // 0 for Ready
      RTS_SndData(0,IconPrintstatus);
	}
    else
    {
      RTS_SndData(0 + CEIconGrap,IconPrintstatus);
    }
    LCD_sd_status = IS_SD_INSERTED(); 		
  }
  else
  {
    SERIAL_ECHOLN("***Initing card fails***");
	if(LanguageRecbuf != 0)
    {
      // 6 for Card Removed
      RTS_SndData(6, IconPrintstatus);
	}
    else
    {
      RTS_SndData(6 + CEIconGrap, IconPrintstatus);
    }
  }
}

void RTSSHOW::RTS_SDCardUpate(void)
{	
  const bool sd_status = IS_SD_INSERTED();
  if (sd_status != LCD_sd_status)
  {
    if (sd_status)
    {
      card.mount();
	  recovery.init();
	  //recovery.changed();
	  recovery.check();
      RTS_SDCardInit();
	  
    }
    else
    {
      card.release();
	  // heating or printing
      if( CardCheckStatus[0] == 1)
      {
        RTS_SDcard_Stop();
		// cancel to check card during printing the gcode file 
        CardCheckStatus[0] = 0;
      }

      if(LanguageRecbuf != 0)
      {
        // 6 for Card Removed
        RTS_SndData(6,IconPrintstatus);
      }
      else
      {
        RTS_SndData(6+CEIconGrap,IconPrintstatus);
      }
      for(int i = 0;i < CardRecbuf.Filesum;i++)
      {
        for(int j = 0;j < 10;j++)
        RTS_SndData(0,CardRecbuf.addr[i]+j);
        // RTS_SndData(4,FilenameIcon+1+i);
        // white
        RTS_SndData((unsigned long)0xFFFF,FilenameNature + (i+1)*16);
      }

      for(int j = 0;j < 10;j++)	
      {
        // clean screen.
        RTS_SndData(0,Printfilename+j);
        // clean filename
        RTS_SndData(0,Choosefilename+j);
      }
      for(int j = 0;j < 8;j++)
      {
        RTS_SndData(0,FilenameCount+j);
      }
      // clean filename Icon
      for(int j = 1;j <= 20;j++)
      {
        RTS_SndData(10,FilenameIcon+j);
        RTS_SndData(10,FilenameIcon1+j);
      }
      memset(&CardRecbuf,0,sizeof(CardRecbuf));
    }
    LCD_sd_status = sd_status;
  }

  // represents to update file list
  if (CardUpdate && LCD_sd_status &&  card.isMounted())
  {
    // clean filename
    for(int j = 0;j < 10;j++)
    {
      RTS_SndData(0,Choosefilename+j);
    }
    for(int j = 0;j < 8;j++)
    {
      RTS_SndData(0,FilenameCount+j);
    }
    for (uint16_t i = 0; i < CardRecbuf.Filesum ; i++) 
    {
      delay(3);
      RTS_SndData(CardRecbuf.Cardshowfilename[i],CardRecbuf.addr[i]);
      RTS_SndData(1,FilenameIcon+1+i);
      RTS_SndData((unsigned long)0xFFFF,FilenameNature + (i+1)*16);		// white
      RTS_SndData(10,FilenameIcon1+1+i);
    }
    CardUpdate = false;
  }
}

int RTSSHOW::RTS_CheckFilement(int mode)
{
  waitway = 4;
  // no filements check
  for(Checkfilenum = 0; ((0 == READ(CHECK_MATWEIAL0)) || (0 == READ(CHECK_MATWEIAL1))) && (Checkfilenum < 50);Checkfilenum ++)
  {
    delay(15);
  }

  if(49 <= Checkfilenum)
  {
    // no filements
    if(mode)
    {
      // for mode status of no filement . the sentence can be canceled, which isn't neccessary?
      FilementStatus[0] = mode;
      if(LanguageRecbuf != 0)
      {
        RTS_SndData(ExchangePageBase + 38, ExchangepageAddr);
      }
      else
      {
        RTS_SndData(ExchangePageBase + 78, ExchangepageAddr);
      }
    }
    Checkfilenum = 0;
    waitway = 0;
    return 1;
  }
  else
  {
    Checkfilenum = 0;
  }
  waitway = 0;
  return 0;	
}

void RTSSHOW::RTS_Init()
{
  Serial2.begin(115200);

  LanguageRecbuf = eeprom_read_byte((unsigned char*)FONT_EEPROM);
   AutoLevelStatus = eeprom_read_byte((unsigned char*)FONT_EEPROM+2);
  if(AutoLevelStatus) 
  {
    RTS_SndData(2, AutoLevelMode);
    // settings.reset();
    queue.enqueue_now_P(PSTR("M420 S0"));
  }
  else 
  {
    bool zig = true;
    int8_t inStart, inStop, inInc;
    showcount = 0;
    RTS_SndData(3, AutoLevelMode);
    settings.load();
    for(int y = 0;y < GRID_MAX_POINTS_Y;y++)
    {
      // away from origin
      if (zig) 
      {
        inStart = 0;
        inStop = GRID_MAX_POINTS_X;
        inInc = 1;
      }
      else
      {
        // towards origin
        inStart = GRID_MAX_POINTS_X - 1;
        inStop = -1;
        inInc = -1;
      }
      zig ^= true;
      for(int x = inStart;x != inStop; x += inInc)
      {
        RTS_SndData(z_values[x][y] *10000, AutolevelVal + showcount*2);
        showcount++;
      }
    }
    queue.enqueue_now_P(PSTR("M420 S1"));
  }
  last_zoffset = probe_offset.z;
  RTS_SndData(probe_offset.z*100, 0x1026);

  VolumeSet = eeprom_read_byte((unsigned char*)FONT_EEPROM+4);
  if(VolumeSet < 0 || VolumeSet > 0xFF)
  {
    VolumeSet = 0x80;
  }

  last_temp_bed = thermalManager.temp_bed.target;
  last_target_temperature[0] = thermalManager.temp_hotend[0].target;
  feedrate_percentage = 100;
  RTS_SndData(feedrate_percentage, FeedrateDisplay);

  /***************turn off motor*****************/
  RTS_SndData(11, FilenameIcon); 

  /***************transmit temperature to screen*****************/
  RTS_SndData(0, NozzlePreheat);
  RTS_SndData(0, BedPreheat);
  RTS_SndData(thermalManager.temp_hotend[0].celsius, NozzleTemp);
  RTS_SndData(thermalManager.temp_bed.celsius, Bedtemp);
  /***************transmit Fan speed to screen*****************/
  #if FAN_COUNT > 0
    // turn off fans
	const uint8_t p = parser.byteval('P');
    thermalManager.set_fan_speed(p, 0);
  #endif
  // turn off fans
  RTS_SndData(2, FanKeyIcon);
  FanStatus = true;

  /*********transmit SD card filename to screen***************/
  RTS_SDCardInit();
  /***************transmit Printer information to screen*****************/
  // clean filename
  for(int j = 0;j < 20;j ++)
  {
    RTS_SndData(0, MacVersion + j);
  }
  char sizebuf[20] = {0};
  sprintf(sizebuf,"%d X %d X %d",MAC_LENGTH, MAC_WIDTH, MAC_HEIGHT);
  RTS_SndData(MACVERSION, MacVersion);
  RTS_SndData(SOFTVERSION, SoftVersion);
  RTS_SndData(sizebuf, PrinterSize);
  RTS_SndData(CORP_WEBSITE, CorpWebsite);

  /**************************some info init*******************************/
  RTS_SndData(0,PrintscheduleIcon);

  /************************clean screen*******************************/
  for(int i = 0;i < MaxFileNumber;i++)
  {
    for(int j = 0;j < 10;j++)
    {
      RTS_SndData(0,SDFILE_ADDR +i*10+j);
    }
  }

  for(int j = 0;j < 10;j++)	
  {
    // clean screen.
    RTS_SndData(0,Printfilename+j);
    // clean filename
    RTS_SndData(0,Choosefilename+j);
  }
  for(int j = 0;j < 8;j++)
  {
    RTS_SndData(0,FilenameCount+j);
  }
  for(int j = 1;j <= MaxFileNumber;j++)
  {
    RTS_SndData(10,FilenameIcon+j);
    RTS_SndData(10,FilenameIcon1+j);
  }
  SERIAL_ECHOLN("===Initing RTS has finished===");
}
int RTSSHOW::RTS_RecData()
{
  while(Serial2.available() > 0 && (recnum < SizeofDatabuf))
  {
    databuf[recnum] = Serial2.read();
    // ignore the invalid data
    if(databuf[0] != FHONE)
    {
      // prevent the program from running.
      if(recnum > 0)
      {
        memset(databuf,0,sizeof(databuf));
        recnum = 0;
      }
      continue;
    }
    delay(10);
    recnum++;
  }

  // receive nothing  	
  if(recnum < 1)
  {
    return -1;
  }
  else if((recdat.head[0] == databuf[0]) && (recdat.head[1] == databuf[1]) && recnum > 2)
  {
    recdat.len = databuf[2];
    recdat.command = databuf[3];
    if(recdat.len == 0x03 && (recdat.command == 0x82 || recdat.command == 0x80) && (databuf[4] == 0x4F) && (databuf[5] == 0x4B))  //response for writing byte
    {   
      memset(databuf,0, sizeof(databuf));
      recnum = 0;
      return -1;
    }
    else if(recdat.command == 0x83)
    {
      // response for reading the data from the variate
      recdat.addr = databuf[4];
      recdat.addr = (recdat.addr << 8 ) | databuf[5];
      recdat.bytelen = databuf[6];
      for(int i = 0;i < recdat.bytelen;i+=2)
      {
        recdat.data[i/2]= databuf[7+i];
        recdat.data[i/2]= (recdat.data[i/2] << 8 )| databuf[8+i];
      }
    }
    else if(recdat.command == 0x81)
    {
      // response for reading the page from the register
      recdat.addr = databuf[4];
      recdat.bytelen = databuf[5];
	  for(int i = 0;i < recdat.bytelen;i++)
	  {
        recdat.data[i]= databuf[6+i];
        // recdat.data[i] = (recdat.data[i] << 8 )| databuf[7+i];
      }
    }
  }
  else
  {
    memset(databuf,0, sizeof(databuf));
    recnum = 0;
	// receive the wrong data
    return -1;
  }
  memset(databuf,0, sizeof(databuf));
  recnum = 0;
  return 2;
}
void RTSSHOW::RTS_SndData(void)
{
  if((snddat.head[0] == FHONE) && (snddat.head[1] == FHTWO) && snddat.len >= 3)
  {
    databuf[0] = snddat.head[0];
    databuf[1] = snddat.head[1];
    databuf[2] = snddat.len;
    databuf[3] = snddat.command;
	// to write data to the register
    if(snddat.command ==0x80)
    {
      databuf[4] = snddat.addr;
      for(int i =0;i <(snddat.len - 2);i++)
      {
        databuf[5+i] = snddat.data[i];
      }
    }
    else if(snddat.len == 3 && (snddat.command ==0x81))
    {
      // to read data from the register
      databuf[4] = snddat.addr;
      databuf[5] = snddat.bytelen;
    }
    else if(snddat.command ==0x82)
    {
      // to write data to the variate
      databuf[4] = snddat.addr >> 8;
      databuf[5] = snddat.addr & 0xFF;
      for(int i =0;i <(snddat.len - 3);i += 2)
      {
        databuf[6 + i] = snddat.data[i/2] >> 8;
        databuf[7 + i] = snddat.data[i/2] & 0xFF;
      }
    }
    else if(snddat.len == 4 && (snddat.command ==0x83))
    {
      // to read data from the variate
      databuf[4] = snddat.addr >> 8;
      databuf[5] = snddat.addr & 0xFF;
      databuf[6] = snddat.bytelen;
    }
    for(int i = 0;i < (snddat.len + 3);i++)
    {
      Serial2.write(databuf[i]);
      delayMicroseconds(1);
    }

    memset(&snddat,0,sizeof(snddat));
    memset(databuf,0, sizeof(databuf));
    snddat.head[0] = FHONE;
    snddat.head[1] = FHTWO;
  }
}
void RTSSHOW::RTS_SndData(const String &s, unsigned long addr, unsigned char cmd /*= VarAddr_W*/)
{
  if(s.length() < 1)
  {
    return;
  }
  RTS_SndData(s.c_str(), addr, cmd);
}

void RTSSHOW::RTS_SndData(const char *str, unsigned long addr, unsigned char cmd/*= VarAddr_W*/)
{
  int len = strlen(str);
  if( len > 0)
  {
    databuf[0] = FHONE;
    databuf[1] = FHTWO;
    databuf[2] = 3+len;
    databuf[3] = cmd;
    databuf[4] = addr >> 8;
    databuf[5] = addr & 0x00FF;
    for(int i =0;i <len ;i++)
    {
      databuf[6 + i] = str[i];
    }

    for(int i = 0;i < (len + 6);i++)
    {
      Serial2.write(databuf[i]);
      delayMicroseconds(1);
    }
    memset(databuf,0, sizeof(databuf));
  }
}

void RTSSHOW::RTS_SndData(char c, unsigned long addr, unsigned char cmd/*= VarAddr_W*/)
{
  snddat.command = cmd;
  snddat.addr = addr;
  snddat.data[0] = (unsigned long)c;
  snddat.data[0] = snddat.data[0] << 8;
  snddat.len = 5;
  RTS_SndData();
}

void RTSSHOW::RTS_SndData(unsigned char* str, unsigned long addr, unsigned char cmd){RTS_SndData((char *)str, addr, cmd);}

void RTSSHOW::RTS_SndData(int n, unsigned long addr, unsigned char cmd/*= VarAddr_W*/)
{
  if(cmd == VarAddr_W )
  {
    if(n > 0xFFFF)
    {
      snddat.data[0] = n >> 16;
      snddat.data[1] = n & 0xFFFF;
      snddat.len = 7;
    }
    else
    {
      snddat.data[0] = n;
      snddat.len = 5;
    }
  }
  else if(cmd == RegAddr_W)
  {
    snddat.data[0] = n;
    snddat.len = 3;
  }
  else if(cmd == VarAddr_R)
  {
    snddat.bytelen = n;
    snddat.len = 4;
  }
  snddat.command = cmd;
  snddat.addr = addr;
  RTS_SndData();
}

void RTSSHOW::RTS_SndData(unsigned int n, unsigned long addr, unsigned char cmd){ RTS_SndData((int)n, addr, cmd); }

void RTSSHOW::RTS_SndData(float n, unsigned long addr, unsigned char cmd){ RTS_SndData((int)n, addr, cmd); }

void RTSSHOW::RTS_SndData(long n, unsigned long addr, unsigned char cmd){ RTS_SndData((unsigned long)n, addr, cmd); }

void RTSSHOW::RTS_SndData(unsigned long n, unsigned long addr, unsigned char cmd/*= VarAddr_W*/)
{
  if(cmd == VarAddr_W )
  {
    if(n > 0xFFFF)
    {
      snddat.data[0] = n >> 16;
      snddat.data[1] = n & 0xFFFF;
      snddat.len = 7;
    }
    else
    {
      snddat.data[0] = n;
      snddat.len = 5;
    }
  }
  else if(cmd == VarAddr_R)
  {
    snddat.bytelen = n;
    snddat.len = 4;
  }
  snddat.command = cmd;
  snddat.addr = addr;
  RTS_SndData();
}
void RTSSHOW::RTS_SDcard_Stop()
{
  waitway = 4;
  card.stopSDPrint();
  queue.clear();
  quickstop_stepper();
  print_job_timer.stop();
  thermalManager.disable_all_heaters();
  print_job_timer.reset();
  #if ENABLED(SDSUPPORT) && ENABLED(POWER_LOSS_RECOVERY)
    card.openJobRecoveryFile(true);
	power_off_commands_count = 0;
  #endif
  const uint8_t p = parser.byteval('P');
  thermalManager.set_fan_speed(p, 0);
  FanStatus = true;

  wait_for_heatup = false;
  PrinterStatusKey[0] = 0;
  G29_status = 0;
  AutohomeKey = false;
  RTS_SndData(11, FilenameIcon); 
  RTS_SndData(0, PrintscheduleIcon);
  RTS_SndData(0, Percentage);
  delay(2);
  for(int j = 0;j < 10;j++)	
  {
    // clean screen.
    RTS_SndData(0,Printfilename+j);
    // clean filename
    RTS_SndData(0,Choosefilename+j);
  }
  for(int j = 0;j < 8;j++)
  {
    RTS_SndData(0,FilenameCount+j);
  }
  InforShowStatus = true;
  TPShowStatus = false;
  if(LanguageRecbuf != 0)
  {
    // 0 for ready
    RTS_SndData(0,IconPrintstatus);
    delay(2);
	// exchange to 1 page
    RTS_SndData(ExchangePageBase + 1, ExchangepageAddr);
  }
  else
  {
    // 0 for ready 
    RTS_SndData(0 + CEIconGrap, IconPrintstatus);
    delay(2);
	// exchange to 45 page
    RTS_SndData(ExchangePageBase + 45, ExchangepageAddr);
  }
  waitway = 0;
}
void RTSSHOW::RTS_HandleData()
{
  int Checkkey = -1;
  // for waiting
  if(waitway > 0)
  {
    memset(&recdat,0 , sizeof(recdat));
    recdat.head[0] = FHONE;
    recdat.head[1] = FHTWO;
    return;
  }
  for(int i = 0;Addrbuf[i] != 0;i++)
  {
    if(recdat.addr == Addrbuf[i])
    {
      if((Addrbuf[i] >= Stopprint) && (Addrbuf[i] <= Resumeprint))
      {
        Checkkey = PrintChoice;
      }
      else if((Addrbuf[i] == NzBdSet) || (Addrbuf[i] == NozzlePreheat) || (Addrbuf[i] == BedPreheat))
      {
        Checkkey = ManualSetTemp;
	  }
      else if((Addrbuf[i] >= AutoZero) && (Addrbuf[i] <= DisplayZaxis))
      {
        Checkkey = XYZEaxis;
      }
      else if((Addrbuf[i] >= FilementUnit1) && (Addrbuf[i] <= FilementUnit2))
      {
        Checkkey = Filement;
      }
	  else if((Addrbuf[i] >= VmaxX) && (Addrbuf[i] <= ABSBed))
	  {
		Checkkey = Advanced;
	  }
      else
      {
        Checkkey = i;
      }
      break;
    }
  }

  if((recdat.addr >= SDFILE_ADDR) && (recdat.addr <= (SDFILE_ADDR + 10 *(FileNum+1))))
  {
    Checkkey = Filename;
  }

  if(Checkkey < 0)
  {
    memset(&recdat,0 , sizeof(recdat));
    recdat.head[0] = FHONE;
    recdat.head[1] = FHTWO;
    return;
  }

  switch(Checkkey)
  {
    case Printfile :
      if(recdat.data[0] == 1)	// card
      {
        InforShowStatus = false;
        CardUpdate = true;
        CardRecbuf.recordcount = -1;
        RTS_SDCardUpate();
        if(LanguageRecbuf != 0)
        {
          // exchange to 2 page
          RTS_SndData(ExchangePageBase + 2, ExchangepageAddr);
	    }
        else
        {
          RTS_SndData(ExchangePageBase + 46, ExchangepageAddr);
        }
      }
      else if(recdat.data[0] == 2)	// return after printing result.
      {
        if(LanguageRecbuf != 0)
        {
          // exchange to 1 page
          RTS_SndData(ExchangePageBase + 1, ExchangepageAddr);
        }
        else
        {
          // exchange to 45 page
          RTS_SndData(ExchangePageBase + 45, ExchangepageAddr);
        }
		InforShowStatus = true;
        TPShowStatus = false;
        quickstop_stepper();
        RTS_SndData(11, FilenameIcon); 
        RTS_SndData(0, PrintscheduleIcon);
        RTS_SndData(0, Percentage);
        delay(10);
        RTS_SndData(0, Timehour);
        RTS_SndData(0, Timemin);
        print_job_timer.reset();
      } 
      else if(recdat.data[0] == 3 )	// Temperature control
      {
        InforShowStatus = true;
        TPShowStatus = false;
        if(LanguageRecbuf != 0)
        {
          if(FanStatus)
          {
            // exchange to 16 page, the fans off
            RTS_SndData(ExchangePageBase + 16, ExchangepageAddr);
          }
          else
          {
            // exchange to 15 page, the fans on
            RTS_SndData(ExchangePageBase + 15, ExchangepageAddr);
          }
        }
        else
        {
          if(FanStatus)
          {
            // exchange to 58 page, the fans off
            RTS_SndData(ExchangePageBase + 58, ExchangepageAddr);
          }
          else
          {
            // exchange to 57 page, the fans on
            RTS_SndData(ExchangePageBase + 57, ExchangepageAddr);
          }
        }
      }
      else if(recdat.data[0] == 4 )	// Settings
      {
        InforShowStatus = false;
      }
      break;

    case Ajust :
      if(recdat.data[0] == 1)
      {
        // InforShowStatus = false;
        FanStatus?RTS_SndData(2, FanKeyIcon):RTS_SndData(3, FanKeyIcon);
      }
      else if(recdat.data[0] == 2)
      {
        InforShowStatus = true;
        // during heating
        if(PrinterStatusKey[1] == 3)
        {
          if(LanguageRecbuf != 0)
          {
            // in heating
            RTS_SndData(ExchangePageBase + 10, ExchangepageAddr);
          }
          else
          {
            RTS_SndData(ExchangePageBase + 52, ExchangepageAddr);
          }
        }
        else if(PrinterStatusKey[1] == 4)
        {
          if(LanguageRecbuf != 0)
          {
            // in the pause
            RTS_SndData(ExchangePageBase + 12, ExchangepageAddr);
          }
          else
          {
            RTS_SndData(ExchangePageBase + 54, ExchangepageAddr); 
          }
        }
        else
        {
          if(LanguageRecbuf != 0)
          {
            // in printing
            RTS_SndData(ExchangePageBase + 11, ExchangepageAddr);
          }
          else
          {
            RTS_SndData(ExchangePageBase + 53, ExchangepageAddr); 
          }
        }
      }
      else if(recdat.data[0] == 3)
      {
        // turn on the fan
        if(FanStatus)
        {
          RTS_SndData(3, FanKeyIcon); 
          const uint8_t p = parser.byteval('P');
		  thermalManager.set_fan_speed(p, 255);
          FanStatus = false;
        }
        else
        {
          // turn off the fan
          RTS_SndData(2, FanKeyIcon); 
          const uint8_t p = parser.byteval('P');
          thermalManager.set_fan_speed(p, 0);
          FanStatus = true;
        }
      }
      break;

    case Feedrate :
      feedrate_percentage = recdat.data[0];
      break;

    case PrintChoice:
      if(recdat.addr == Stopprint)
      { 
        if(PrintStatue[1] == 1 && recdat.data[0] == 0xF0)	// in the pause
        {
          if(LanguageRecbuf != 0)
          {
            RTS_SndData(ExchangePageBase + 12, ExchangepageAddr); // exchange to 12 page
		  }
          else
          {
            RTS_SndData(ExchangePageBase + 54, ExchangepageAddr);
          }
          break;
        }
        else if(PrintStatue[1] == 0 && recdat.data[0] == 0xF0)	
        {
          // during heating
          if(PrinterStatusKey[1] == 3)
          {
            if(LanguageRecbuf != 0)
            {
              // in heating
              RTS_SndData(ExchangePageBase + 10, ExchangepageAddr);
            }
            else
            {
              RTS_SndData(ExchangePageBase + 52, ExchangepageAddr);
            }
          }
          else
          {
            if(LanguageRecbuf != 0)
            {
              // in printing
              RTS_SndData(ExchangePageBase + 11, ExchangepageAddr);
			}
            else
            {
              RTS_SndData(ExchangePageBase + 53, ExchangepageAddr);
            }
          }
          break;
        }
        // recover the status waiting to check filements
        // FilementStatus[0]  =  0;

        if(LanguageRecbuf != 0)
          RTS_SndData(ExchangePageBase + 86, ExchangepageAddr);	
        else
          RTS_SndData(ExchangePageBase + 87, ExchangepageAddr); 
        RTS_SndData(0,Timehour);
        RTS_SndData(0,Timemin);
        Update_Time_Value = 0;
        // close the key of checking card in printing
        CardCheckStatus[0] = 0;
        tempbed = 0;
        temphot = 0;
		recovery.purge();
        RTS_SDcard_Stop();
      }
      else if(recdat.addr == Pauseprint)
      {
        if(recdat.data[0] != 0xF1)
		{
          break;
        }

        if(LanguageRecbuf != 0)
          RTS_SndData(ExchangePageBase + 86, ExchangepageAddr);	
        else
          RTS_SndData(ExchangePageBase + 87, ExchangepageAddr); 

        // reject to receive cmd
        waitway = 1;
        pause_z = current_position[Z_AXIS];
        pause_e = current_position[E_AXIS] - 10;
        card.pauseSDPrint();
        print_job_timer.pause();
        if(!temphot)
          temphot = thermalManager.degTargetHotend(0);
        if(!tempbed)
          tempbed = thermalManager.degTargetBed();
        // thermalManager.setTargetHotend(0, 0);
        thermalManager.setTargetBed(0);
        // for return the corresponding page
        PrintStatue[1] = 1;
        PrinterStatusKey[1] = 4;
        Update_Time_Value = 0;
        queue.inject_P(PSTR("G28 X0 Y0"));
      }
      else if((recdat.addr == Resumeprint) && (recdat.data[0] == 1))
      {
        #if CHECKFILEMENT
          /**************checking filement status during printing************/
         if(RTS_CheckFilement(0)) 
          {
            if(thermalManager.temp_hotend[0].target >= 185)
            {
              for(startprogress=0;startprogress < 5;startprogress++)
              {
                RTS_SndData(startprogress, ExchFlmntIcon);
                delay(400);
              }
            }
            else
            {
              if(LanguageRecbuf != 0)
                rtscheck.RTS_SndData(ExchangePageBase + 44, ExchangepageAddr);
              else
                rtscheck.RTS_SndData(ExchangePageBase + 81, ExchangepageAddr);
            }
            break;
          }
        #endif

        if(LanguageRecbuf != 0)
          RTS_SndData(ExchangePageBase + 86, ExchangepageAddr);	
        else
          RTS_SndData(ExchangePageBase + 87, ExchangepageAddr); 

        char pause_str_Z[16];
        char pause_str_E[16];
        memset(pause_str_Z, 0, sizeof(pause_str_Z));
        dtostrf(pause_z, 3, 2, pause_str_Z);
        memset(pause_str_E, 0, sizeof(pause_str_E));
        dtostrf(pause_e, 3, 2, pause_str_E);

        memset(commandbuf,0,sizeof(commandbuf));
        sprintf_P(commandbuf, PSTR("M190 S%i"), tempbed);
        queue.enqueue_one_now(commandbuf);
        memset(commandbuf,0,sizeof(commandbuf));
        sprintf_P(commandbuf, PSTR("M109 S%i"), temphot);
        queue.enqueue_one_now(commandbuf);
        memset(commandbuf,0,sizeof(commandbuf));
        sprintf_P(commandbuf, PSTR("G0 Z%s"), pause_str_Z);
        queue.enqueue_one_now(commandbuf);
        sprintf_P(commandbuf, PSTR("G92 E%s"), pause_str_E);
        queue.enqueue_one_now(commandbuf);
        tempbed = 0;
        temphot = 0;

        card.startFileprint();
        print_job_timer.start();

        if(LanguageRecbuf != 0)
          RTS_SndData(1,IconPrintstatus);
        else
          RTS_SndData(1+CEIconGrap,IconPrintstatus);
        PrintStatue[1] = 0;
        Update_Time_Value = 0;
        PrinterStatusKey[1] = 3;
        // open the key of checking card in printing
        CardCheckStatus[0] = 1;
        if(LanguageRecbuf != 0)
        {
          // exchange to 10 page
          RTS_SndData(ExchangePageBase + 10, ExchangepageAddr);
		}
        else
        {
          RTS_SndData(ExchangePageBase + 52, ExchangepageAddr); 
        }
      }
      if(recdat.addr == Resumeprint && recdat.data[0] == 2)
      { 
        NozzleTempStatus[2] = 1;
        thermalManager.temp_hotend[0].target = temphot;
        startprogress  = 0;
        if(LanguageRecbuf != 0)
          RTS_SndData(ExchangePageBase + 7, ExchangepageAddr);
        else
          RTS_SndData(ExchangePageBase + 82, ExchangepageAddr);
      }
      break;

    case Zoffset:
	  if(recdat.data[0]>= 32768)
      {
        probe_offset.z = ((float)recdat.data[0]-65536)/100;
      }
      else
      {
        probe_offset.z = ((float)recdat.data[0])/100;
      }
      if (WITHIN((probe_offset.z), Z_PROBE_OFFSET_RANGE_MIN, Z_PROBE_OFFSET_RANGE_MAX)) 
      {
        babystep.add_steps(Z_AXIS, (BABYSTEP_MULTIPLICATOR_Z) * (probe_offset.z - last_zoffset));
      }
      break;

    case TempControl:
      if(recdat.data[0] == 0)
      {
        InforShowStatus = true;
        TPShowStatus = false;
      }
      else if(recdat.data[0] == 1)
      {

      }
      else if(recdat.data[0] == 2)
      {
        // InforShowStatus = true;
      }
      else if(recdat.data[0] == 3)
      {
        // turn on the fan
        if(FanStatus)
        {
          const uint8_t p = parser.byteval('P');		 
		  thermalManager.set_fan_speed(p, 255);
          FanStatus = false;
          if(LanguageRecbuf != 0)
          {
            //exchange to 15 page, the fans on
            RTS_SndData(ExchangePageBase + 15, ExchangepageAddr);
          }
          else
          {
            // exchange to 57 page, the fans on
            RTS_SndData(ExchangePageBase + 57, ExchangepageAddr);
          }
        }
        else
        {
          // turn off the fan
          const uint8_t p = parser.byteval('P');
          thermalManager.set_fan_speed(p, 0);
          FanStatus = true;
          if(LanguageRecbuf != 0)
          {
            // exchange to 16 page, the fans off
            RTS_SndData(ExchangePageBase + 16, ExchangepageAddr);
          }
          else
          {
            // exchange to 58 page, the fans on
            RTS_SndData(ExchangePageBase + 58, ExchangepageAddr);
          }
        }
      }
      else if(recdat.data[0] == 5)	// PLA mode
      {
        thermalManager.setTargetHotend(ui.preheat_hotend_temp[0], 0);
        thermalManager.setTargetBed(ui.preheat_bed_temp[0]);

        RTS_SndData(ui.preheat_hotend_temp[0], NozzlePreheat);
        RTS_SndData(ui.preheat_bed_temp[0], BedPreheat);
      }
      else if(recdat.data[0] == 6)	// ABS mode
      {
        thermalManager.setTargetHotend((ui.preheat_hotend_temp[1]), 0);
        thermalManager.setTargetBed(ui.preheat_bed_temp[1]);
        RTS_SndData(ui.preheat_hotend_temp[1], NozzlePreheat);
        RTS_SndData(ui.preheat_bed_temp[1], BedPreheat);
      }
      else if(recdat.data[0] == 0xF1)
      {
        // InforShowStatus = true;
        const uint8_t p = parser.byteval('P');		  
		thermalManager.set_fan_speed(p, 255);
        FanStatus = false;
        thermalManager.disable_all_heaters();

        RTS_SndData(0, NozzlePreheat);
        delay(1);
        RTS_SndData(0, BedPreheat);
        delay(1);
        if(LanguageRecbuf != 0)
        {
          // 8 for Cooling
          RTS_SndData(8,IconPrintstatus);
          RTS_SndData(ExchangePageBase + 15, ExchangepageAddr);
        }
        else
        {
          RTS_SndData(8+CEIconGrap,IconPrintstatus);
          RTS_SndData(ExchangePageBase + 57, ExchangepageAddr);
        }
        PrinterStatusKey[1] = 2;
      }
      break;

    case ManualSetTemp:
      if(recdat.addr == NzBdSet)
      {
        if(recdat.data[0] == 0)
        {
          if(LanguageRecbuf != 0)
          {
            if(FanStatus)
            {
              // exchange to 16 page, the fans off
              RTS_SndData(ExchangePageBase + 16, ExchangepageAddr);
            }
            else
            {
              // exchange to 15 page, the fans on
              RTS_SndData(ExchangePageBase + 15, ExchangepageAddr);
            }
          }
          else
          {
            if(FanStatus)
            {
              // exchange to 58 page, the fans off
              RTS_SndData(ExchangePageBase + 58, ExchangepageAddr);
            }
            else
            {
              // exchange to 57 page, the fans on
              RTS_SndData(ExchangePageBase + 57, ExchangepageAddr);
            }
          }
        }
        else if(recdat.data[0] == 1)
        {
          thermalManager.temp_hotend[0].target = 0;
          RTS_SndData(0, NozzlePreheat);
        }
        else if(recdat.data[0] == 2)
        {
          thermalManager.temp_bed.target = 0;
          RTS_SndData(0, BedPreheat);
        }
      }
      else if(recdat.addr == NozzlePreheat)
      {
        thermalManager.temp_hotend[0].target = recdat.data[0];
        thermalManager.setTargetHotend(thermalManager.temp_hotend[0].target, 0);
      }
      else if(recdat.addr == BedPreheat)
      {
        thermalManager.temp_bed.target = recdat.data[0];
        thermalManager.setTargetBed(thermalManager.temp_bed.target);
      }
      break;

    case Setting:
      if(recdat.data[0] == 0)	// return to main page
      {
        InforShowStatus = true;
        TPShowStatus = false;
      }
      else if(recdat.data[0] == 1)	//Bed Autoleveling
      {
        AutoLevelStatus?RTS_SndData(2, AutoLevelMode):RTS_SndData(3, AutoLevelMode);
        // Motor Icon
        RTS_SndData(10, FilenameIcon);
        // only for prohibiting to receive massage
        waitway = 6;
        InforShowStatus = AutohomeKey = true;
        AutoHomeIconNum = 0;
        Update_Time_Value = 0;
        queue.enqueue_now_P(PSTR("G28"));

        queue.enqueue_now_P(PSTR("G1 F200 Z0.0")); 
        if(LanguageRecbuf != 0)
          RTS_SndData(ExchangePageBase + 32, ExchangepageAddr);
        else
          RTS_SndData(ExchangePageBase + 74, ExchangepageAddr); 
      }
      else if(recdat.data[0] == 2)
      {
        // Exchange filement
        InforShowStatus = true;
        TPShowStatus = false;
        memset(ChangeMaterialbuf, 0, sizeof(ChangeMaterialbuf));
        ChangeMaterialbuf[1]=ChangeMaterialbuf[0] = 10;
        // It's ChangeMaterialbuf for show,instead of current_position[E_AXIS] in them.
        RTS_SndData(10*ChangeMaterialbuf[0], FilementUnit1);
        RTS_SndData(10*ChangeMaterialbuf[1], FilementUnit2);
        RTS_SndData(thermalManager.temp_hotend[0].celsius,NozzleTemp);
        thermalManager.setTargetHotend(thermalManager.temp_hotend[0].target, 0);
        RTS_SndData(thermalManager.temp_hotend[0].target,NozzlePreheat);
        delay(2);
        if(LanguageRecbuf != 0)
        {
          // Change filement
          RTS_SndData(ExchangePageBase + 23, ExchangepageAddr);
        }
        else
        {
          RTS_SndData(ExchangePageBase + 65, ExchangepageAddr); 
        }
      }
      else if(recdat.data[0] == 3)	// Move
      {
        // InforShowoStatus = false;
        AxisPagenum = 0;
        rtscheck.RTS_SndData(10*current_position[X_AXIS], DisplayXaxis);
        rtscheck.RTS_SndData(10*current_position[Y_AXIS], DisplayYaxis);
        rtscheck.RTS_SndData(10*current_position[Z_AXIS], DisplayZaxis);
        delay(2);
        if(LanguageRecbuf != 0)
        {
          // Move axis, 10mm
          RTS_SndData(ExchangePageBase + 29, ExchangepageAddr);
        }
        else
        {
          RTS_SndData(ExchangePageBase + 71, ExchangepageAddr); 
        }
      }
      else if(recdat.data[0] == 4)	// Language
      {

      }
      else if(recdat.data[0] == 5)	// Advanced
      {
        InforShowStatus = false;
      }
      else if(recdat.data[0] == 6)	// Diabalestepper
      {
        queue.enqueue_now_P(PSTR("M84"));
        RTS_SndData(11, FilenameIcon); 
      }
	  else if(recdat.data[0] == 7)	// return to main page
      {
    
      }
      else if(recdat.data[0] == 8)	//velocity
      {
		RTS_SndData(planner.settings.max_feedrate_mm_s[X_AXIS], VmaxX);
        RTS_SndData(planner.settings.max_feedrate_mm_s[Y_AXIS], VmaxY);
	    RTS_SndData(planner.settings.max_feedrate_mm_s[Z_AXIS], VmaxZ);
        RTS_SndData(planner.settings.max_feedrate_mm_s[E_AXIS], VmaxE);  
        if(LanguageRecbuf != 0)
        {
          // Change filement
          RTS_SndData(ExchangePageBase + 92, ExchangepageAddr);
        }
        else
        {
          RTS_SndData(ExchangePageBase + 102, ExchangepageAddr); 
        }
      }
      else if(recdat.data[0] == 9)  //Acceleration
      {
        RTS_SndData(planner.settings.acceleration, Accel);
        RTS_SndData(planner.settings.retract_acceleration, ARetract);
        RTS_SndData(planner.settings.travel_acceleration, ATravel);
        RTS_SndData(planner.settings.max_acceleration_mm_per_s2[A_AXIS], AmaxX);
		RTS_SndData(planner.settings.max_acceleration_mm_per_s2[B_AXIS], AmaxY);
		RTS_SndData(planner.settings.max_acceleration_mm_per_s2[C_AXIS], AmaxZ);
		RTS_SndData(planner.settings.max_acceleration_mm_per_s2[E_AXIS], AmaxE);
        if(LanguageRecbuf != 0)
        {
          RTS_SndData(ExchangePageBase + 93, ExchangepageAddr);
        }
        else
        {
          RTS_SndData(ExchangePageBase + 103, ExchangepageAddr); 
        }
      }
      else if(recdat.data[0] == 10)	// jerk
      {
        RTS_SndData(10*planner.max_jerk[X_AXIS], Xjerk);
        RTS_SndData(10*planner.max_jerk[Y_AXIS], Yjerk);
        RTS_SndData(10*planner.max_jerk[Z_AXIS], Zjerk);
		RTS_SndData(10*planner.max_jerk[E_AXIS], Ejerk);
        if(LanguageRecbuf != 0)
        {
          RTS_SndData(ExchangePageBase + 94, ExchangepageAddr);
        }
        else
        {
          RTS_SndData(ExchangePageBase + 104, ExchangepageAddr); 
        }
      }
      else if(recdat.data[0] == 11)	// steps
      {
		RTS_SndData(10*planner.settings.axis_steps_per_mm[A_AXIS], Xsteps);
        RTS_SndData(10*planner.settings.axis_steps_per_mm[B_AXIS], Ysteps);
        RTS_SndData(10*planner.settings.axis_steps_per_mm[C_AXIS], Zsteps);
		RTS_SndData(10*planner.settings.axis_steps_per_mm[E_AXIS_N(active_extruder)], Esteps);
        if(LanguageRecbuf != 0)
        {
          RTS_SndData(ExchangePageBase + 95, ExchangepageAddr);
        }
        else
        {
          RTS_SndData(ExchangePageBase + 105, ExchangepageAddr); 
        }
      }
	  else if(recdat.data[0] == 12)	// preheat
      {
		RTS_SndData(ui.preheat_hotend_temp[0], PLANozzle);
		RTS_SndData(ui.preheat_bed_temp[0], PLABed);
		RTS_SndData(ui.preheat_hotend_temp[1], ABSNozzle);
		RTS_SndData(ui.preheat_bed_temp[1], ABSBed); 
        if(LanguageRecbuf != 0)
        {
          RTS_SndData(ExchangePageBase + 96, ExchangepageAddr);
        }
        else
        {
          RTS_SndData(ExchangePageBase + 106, ExchangepageAddr); 
        }
      }
      else if(recdat.data[0] == 13)	// Printer Information
      {
        RTS_SndData(CORP_WEBSITE, CorpWebsite);
      }
	  else if(recdat.data[0] == 0xF1)
      {
		settings.init_eeprom();
      }
      else if(recdat.data[0] == 0xF0)	// not to cancel heating
      {
        break;
      }
      break;

    case ReturnBack:
      if(recdat.data[0] == 1)	 // return to the tool page
      {
        Update_Time_Value = RTS_UPDATE_VALUE;
        InforShowStatus = false;
        if(last_zoffset != probe_offset.z)
        {
          last_zoffset = probe_offset.z;
          RTS_SndData(probe_offset.z*100, 0x1026);
          settings.save();
        }
        if(LanguageRecbuf != 0)
        {
          RTS_SndData(ExchangePageBase + 21, ExchangepageAddr);
        }
        else
        {
          RTS_SndData(ExchangePageBase + 63, ExchangepageAddr);
        }
      }
	  if(recdat.data[0] == 2)
      {
        // return to the Level mode page
        if(LanguageRecbuf != 0)
          RTS_SndData(ExchangePageBase + 22, ExchangepageAddr); 
        else
          RTS_SndData(ExchangePageBase + 64, ExchangepageAddr); 
      }
      break;

    case Bedlevel:
      if(recdat.data[0] == 1)
      {
		// Z-axis to home
        planner.synchronize();
        // only for prohibiting to receive massage
        waitway = 6;
        InforShowStatus = false;
        AutohomeKey = true;
        if (!TEST(axis_known_position, X_AXIS) || !TEST(axis_known_position, Y_AXIS))
        {
          queue.enqueue_now_P(PSTR("G28"));
        }
        else
        {
          queue.enqueue_now_P(PSTR("G28 Z0"));
        }
      }
      else if(recdat.data[0] == 2)
      {
        if (WITHIN((probe_offset.z + 0.1), Z_PROBE_OFFSET_RANGE_MIN, Z_PROBE_OFFSET_RANGE_MAX)) 
        {
          babystep.add_steps(Z_AXIS, BABYSTEP_MULTIPLICATOR_Z);
          probe_offset.z = (probe_offset.z + 0.1);
        }
      }
      else if(recdat.data[0] == 3)
      {
        if (WITHIN((probe_offset.z - 0.1), Z_PROBE_OFFSET_RANGE_MIN, Z_PROBE_OFFSET_RANGE_MAX)) 
        {
          babystep.add_steps(Z_AXIS, -BABYSTEP_MULTIPLICATOR_Z);
          probe_offset.z = (probe_offset.z - 0.1);
        }
      }
      else if(recdat.data[0] == 4)
      {
        // Assitant Level
        waitway = 4;
        queue.enqueue_now_P(PSTR("G28"));
        queue.enqueue_now_P(PSTR("G90"));

        if(LanguageRecbuf != 0)
          RTS_SndData(ExchangePageBase + 28, ExchangepageAddr); 
        else
          RTS_SndData(ExchangePageBase + 84, ExchangepageAddr); 
      }
      else if(recdat.data[0] == 5)
      {
        // AutoLevel
        waitway = 3;
        RTS_SndData(1, AutolevelIcon); 
        if(LanguageRecbuf != 0)
          RTS_SndData(ExchangePageBase + 43, ExchangepageAddr);
        else
          RTS_SndData(ExchangePageBase + 85, ExchangepageAddr); 
        queue.enqueue_now_P(PSTR("G29")); 
        // planner.synchronize();
      }
      else if(recdat.data[0] == 6)
      {
        // Assitant Level ,  Centre 1
        waitway = 4;
        queue.enqueue_now_P(PSTR("G1 F100 Z3")); 
        queue.enqueue_now_P(PSTR("G1 X150 Y125 F3600"));
        queue.enqueue_now_P(PSTR("G1 F100 Z0"));
        waitway = 0;
      }
      else if(recdat.data[0] == 7)
      {
        // Assitant Level , Front Left 2
        waitway = 4;
        queue.enqueue_now_P(PSTR("G1 F100 Z3")); 
        queue.enqueue_now_P(PSTR("G1 X30 Y25 F3600"));
        queue.enqueue_now_P(PSTR("G1 F100 Z0"));
        waitway = 0;
      }
      else if(recdat.data[0] == 8)
      {
        // Assitant Level , Front Right 3
        waitway = 4;
        queue.enqueue_now_P(PSTR("G1 F100 Z3")); 
        queue.enqueue_now_P(PSTR("G1 X270 Y25 F3600"));
        queue.enqueue_now_P(PSTR("G1 F100 Z0"));
        waitway = 0;
      }
      else if(recdat.data[0] == 9)
      {
        // Assitant Level , Back Right 4
        waitway = 4;
        queue.enqueue_now_P(PSTR("G1 F100 Z3")); 
        queue.enqueue_now_P(PSTR("G1 X270 Y225 F3600"));
        queue.enqueue_now_P(PSTR("G1 F100 Z0"));
        waitway = 0;
      }
      else if(recdat.data[0] == 10)
      {
        // Assitant Level , Back Left 5
        waitway = 4;
        queue.enqueue_now_P(PSTR("G1 F100 Z3")); 
        queue.enqueue_now_P(PSTR("G1 X30 Y225 F3600"));
        queue.enqueue_now_P(PSTR("G1 F100 Z0"));
        waitway = 0;
      }
	  else if(recdat.data[0] == 11)
      {
        // Autolevel switch
        if(AutoLevelStatus)
        {
          // turn on the Autolevel
          RTS_SndData(3, AutoLevelMode);	
          AutoLevelStatus = false;
          queue.enqueue_now_P(PSTR("M420 S1"));
          // settings.load();
        }
        else
        {
          // turn off the Autolevel
          RTS_SndData(2, AutoLevelMode);
          AutoLevelStatus = true;
          queue.enqueue_now_P(PSTR("M420 S0"));
          // settings.reset();
        }
        last_zoffset = probe_offset.z;
        RTS_SndData(probe_offset.z*100, 0x1026); 
        eeprom_write_byte((unsigned char*)FONT_EEPROM+2, AutoLevelStatus);
      }

      RTS_SndData(10, FilenameIcon); 
      break;

    case XYZEaxis:
      AxisEnum axis;
      float min,max;
      waitway = 4;
      if(recdat.addr == DisplayXaxis)
      {
        axis = X_AXIS;
        min = X_MIN_POS;
        max = X_MAX_POS;
      }
      else if(recdat.addr == DisplayYaxis)
      {
        axis = Y_AXIS;
        min = Y_MIN_POS;
        max = Y_MAX_POS;
      }
      else if(recdat.addr == DisplayZaxis)
      {
        axis = Z_AXIS;
        min = Z_MIN_POS;
        max = Z_MAX_POS;
      }
      else if(recdat.addr == AutoZero)
      {
        // autohome
        if(recdat.data[0] == 3)
        {
          waitway = 4;
          queue.enqueue_now_P(PSTR("G28"));
          InforShowStatus = AutohomeKey = true;
          AutoHomeIconNum = 0;
          Update_Time_Value = 0;
          if(LanguageRecbuf != 0)
          {
            RTS_SndData(ExchangePageBase + 32, ExchangepageAddr);
          }
          else
          {
            RTS_SndData(ExchangePageBase + 74, ExchangepageAddr); 
          }
          RTS_SndData(10,FilenameIcon);
        }
        else
        {
          AxisPagenum = recdat.data[0];
          waitway = 0;
        }
        break;
      }

      current_position[axis] = ((float)recdat.data[0])/10;
      if (current_position[axis] < min) current_position[axis] = min;
      else if (current_position[axis] > max) current_position[axis] = max;

      RTS_line_to_current(axis);
      RTS_SndData(10*current_position[X_AXIS], DisplayXaxis);
      RTS_SndData(10*current_position[Y_AXIS], DisplayYaxis);
      RTS_SndData(10*current_position[Z_AXIS], DisplayZaxis);
      delay(1);
      RTS_SndData(10, FilenameIcon); 
      waitway = 0;
      break;

    case Filement:
      #if CHECKFILEMENT
        /**************checking filement status during changing filement************/
        if(RTS_CheckFilement(3)) break;
      #endif

      unsigned int IconTemp;
      if(recdat.addr == Exchfilement)
      {
        if(recdat.data[0] == 1)
        {
          original_extruder = active_extruder;
          active_extruder = 0;
          current_position[E_AXIS] -= ChangeMaterialbuf[0];

          if( NozzleTempStatus[1]== 0 && thermalManager.temp_hotend[0].celsius < (PLA_ABSModeTemp-5))
          {
            NozzleTempStatus[1] = 1; 
            RTS_SndData((int)PLA_ABSModeTemp, 0x1020);
            delay(5);
            if(LanguageRecbuf != 0)
            {
              // exchange to 24 page
              RTS_SndData(ExchangePageBase + 24, ExchangepageAddr);
            }
            else
            {
              RTS_SndData(ExchangePageBase + 66, ExchangepageAddr);
            }
            break;
          }
        }
        else if(recdat.data[0] == 2)
        {
          original_extruder = active_extruder;
          active_extruder = 0;
          current_position[E_AXIS] += ChangeMaterialbuf[0];

          if( NozzleTempStatus[1]== 0 && thermalManager.temp_hotend[0].celsius < (PLA_ABSModeTemp-5))
          {
            NozzleTempStatus[1] = 1; 
            RTS_SndData((int)PLA_ABSModeTemp, 0x1020);
            delay(5);
            if(LanguageRecbuf != 0)
            {
              // exchange to 24 page
              RTS_SndData(ExchangePageBase + 24, ExchangepageAddr);
            }
            else
            {
              RTS_SndData(ExchangePageBase + 66, ExchangepageAddr); 
            }
            break;
          }
        }
        else if(recdat.data[0] == 3)
        {
          original_extruder = active_extruder;
          active_extruder = 1;
          current_position[E_AXIS] -= ChangeMaterialbuf[1];

          if( NozzleTempStatus[1]== 0 && thermalManager.temp_hotend[0].celsius < (PLA_ABSModeTemp-5))
          {
            NozzleTempStatus[1] = 1; 
            RTS_SndData((int)PLA_ABSModeTemp, 0x1020);
            delay(5);
            if(LanguageRecbuf != 0)
            {
              // exchange to 24 page
              RTS_SndData(ExchangePageBase + 24, ExchangepageAddr);
            }
            else
            {
              RTS_SndData(ExchangePageBase + 66, ExchangepageAddr);
            }
            break;
          }
        }
        else if(recdat.data[0] == 4)
        {
          original_extruder = active_extruder;
          active_extruder = 1;
          current_position[E_AXIS] += ChangeMaterialbuf[1];

          if( NozzleTempStatus[1]== 0 && thermalManager.temp_hotend[0].celsius < (PLA_ABSModeTemp-5))
          {
            NozzleTempStatus[1] = 1; 
            RTS_SndData((int)PLA_ABSModeTemp, 0x1020);
            delay(5);
            if(LanguageRecbuf != 0)
            {
              // exchange to 24 page
              RTS_SndData(ExchangePageBase + 24, ExchangepageAddr);
            }
            else
            {
              RTS_SndData(ExchangePageBase + 66, ExchangepageAddr);
            }
            break;
          }
        }
        else if(recdat.data[0] == 5)
        {
          NozzleTempStatus[0] = 1;
          // InforShowoStatus = true;

          thermalManager.temp_hotend[0].target = (thermalManager.temp_hotend[0].target >= PLA_ABSModeTemp? thermalManager.temp_hotend[0].target:  PLA_ABSModeTemp);
          IconTemp = thermalManager.temp_hotend[0].celsius * 100/thermalManager.temp_hotend[0].target;
          if(IconTemp >= 100)
          {
            IconTemp = 100;
          }
          RTS_SndData(IconTemp, HeatPercentIcon);

          RTS_SndData(thermalManager.temp_hotend[0].celsius, NozzleTemp);
          thermalManager.setTargetHotend(thermalManager.temp_hotend[0].target, 0);
          RTS_SndData(thermalManager.temp_hotend[0].target, NozzlePreheat);
          delay(5);
          if(LanguageRecbuf != 0)
          {
            // exchange to 26 page
            RTS_SndData(ExchangePageBase + 26, ExchangepageAddr);
          }
          else
          {
            RTS_SndData(ExchangePageBase + 68, ExchangepageAddr);
          }
          break;
        }
        else if(recdat.data[0] == 6)
        {
          NozzleTempStatus[1] = 0;
          if(LanguageRecbuf != 0)
          {
            // exchange to 23 page
            RTS_SndData(ExchangePageBase + 23, ExchangepageAddr);
          }
          else
          {
            RTS_SndData(ExchangePageBase + 65, ExchangepageAddr);
          }
          break;
        }
        else if(recdat.data[0] == 0xF1)
        {
          // InforShowoStatus = true;
          NozzleTempStatus[0] = NozzleTempStatus[1] = 0;
		  thermalManager.disable_all_heaters();
		  RTS_SndData(0, NozzlePreheat);
          delay(1);
          if(LanguageRecbuf != 0)
          {
            RTS_SndData(ExchangePageBase + 23, ExchangepageAddr);
          }
          else
          {
            RTS_SndData(ExchangePageBase + 65, ExchangepageAddr);
          }
          break;
        }
        else if(recdat.data[0] == 0xF0)	// not to cancel heating
        {
          break;
        }

        RTS_line_to_current(E_AXIS);
        // It's ChangeMaterialbuf for show,instead of current_position[E_AXIS] in them.
        RTS_SndData(10*ChangeMaterialbuf[0], FilementUnit1);
        RTS_SndData(10*ChangeMaterialbuf[1], FilementUnit2);
        active_extruder = original_extruder;
      }
      else if(recdat.addr == FilementUnit1)
      {
        ChangeMaterialbuf[0] = ((float)recdat.data[0])/10;
      }
      else if(recdat.addr == FilementUnit2)
      {
        ChangeMaterialbuf[1] = ((float)recdat.data[0])/10;
      }
      break;

    case LanguageChoice:
      LanguageRecbuf = recdat.data[0];
      eeprom_write_byte((unsigned char*)FONT_EEPROM, LanguageRecbuf);
      //next step:record the data to EEPROM
      if(card.isMounted())
      {
        if(LanguageRecbuf != 0)
        {
          // 0 for Ready
          RTS_SndData(0,IconPrintstatus);
        }
        else
        {
          RTS_SndData(0+CEIconGrap,IconPrintstatus);
        }
      }
      else
      {
        if(LanguageRecbuf != 0)
        {
          // 6 for Card Removed
          RTS_SndData(6,IconPrintstatus);
        }
        else
        {
          RTS_SndData(6+CEIconGrap,IconPrintstatus);
        }
      }
      break;

    case No_Filement:
      char comdbuf[30];
      if(recdat.data[0] == 1)
      {
        //checking filement status during changing filement
        if(RTS_CheckFilement(0)) break;

        // check filement before starting to print
        if(FilementStatus[0] == 1)
        {
          queue.enqueue_one_now(cmdbuf);
          queue.enqueue_now_P(PSTR("M24"));
          // clean screen.
          for(int j = 0;j < 10;j++)
          {
            RTS_SndData(0,Printfilename+j);
          }

          int filelen = strlen(CardRecbuf.Cardshowfilename[FilenamesCount]);
          filelen = (TEXTBYTELEN - filelen)/2;
          if(filelen > 0)
          {
            char buf[20];
            memset(buf,0,sizeof(buf));
            strncpy(buf,"         ",filelen);
            strcpy(&buf[filelen],CardRecbuf.Cardshowfilename[FilenamesCount]);
            RTS_SndData(buf, Printfilename);
          }
          else
          {
            RTS_SndData(CardRecbuf.Cardshowfilename[FilenamesCount], Printfilename);
          }
          delay(2);
          if(LanguageRecbuf != 0)
          {
            // 1 for Heating
            RTS_SndData(1,IconPrintstatus);
            delay(2);
            // exchange to 10 page
            RTS_SndData(ExchangePageBase + 10, ExchangepageAddr);
          }
          else
          {
            // 1 for Heating
            RTS_SndData(1+CEIconGrap,IconPrintstatus);
            delay(2);
            RTS_SndData(ExchangePageBase + 52, ExchangepageAddr); 
          }

          TPShowStatus = InforShowStatus = true;
          PrinterStatusKey[0] = 1;
          PrinterStatusKey[1] = 3;
          // open the key of checking card in printing
          CardCheckStatus[0] = 1;
          FilenamesCount = PrintStatue[1] = 0;
          // recover the status waiting to check filements
          FilementStatus[0]  =  0;
        }
        else if(FilementStatus[0] == 2)
        {
          // check filements status during printing
          char pause_str_Z[16];
          char pause_str_E[16];
          memset(pause_str_Z, 0, sizeof(pause_str_Z));
          dtostrf(pause_z, 3, 2, pause_str_Z);
          memset(pause_str_E, 0, sizeof(pause_str_E));
          dtostrf(pause_e, 3, 2, pause_str_E);
          sprintf_P(comdbuf, PSTR("M190 S%i"), tempbed);
          queue.enqueue_one_now(comdbuf);
          memset(comdbuf,0,sizeof(comdbuf));
          sprintf_P(comdbuf, PSTR("M109 S%i"), temphot);
          queue.enqueue_one_now(comdbuf);
          memset(comdbuf,0,sizeof(comdbuf));
          sprintf_P(comdbuf, PSTR("G0 Z%s"), pause_str_Z);
          queue.enqueue_one_now(comdbuf);
          sprintf_P(comdbuf, PSTR("G92 E%s"), pause_str_E);
          queue.enqueue_one_now(comdbuf);
          tempbed = 0;
          temphot = 0;

          card.startFileprint();
          print_job_timer.start();

          if(LanguageRecbuf != 0)
          {
            // 1 for Heating
            RTS_SndData(1,IconPrintstatus);
          }
          else
          {
            RTS_SndData(1+CEIconGrap,IconPrintstatus);
          }
          PrintStatue[1] = 0;
          PrinterStatusKey[1] = 3;
          // open the key of checking card in printing
          CardCheckStatus[0] = 1;
          if(LanguageRecbuf != 0)
          {
            // exchange to 10 page
            RTS_SndData(ExchangePageBase + 10, ExchangepageAddr);
          }
          else
          {
            RTS_SndData(ExchangePageBase + 52, ExchangepageAddr);
          }

          // recover the status waiting to check filements
          FilementStatus[0]  =  0;
        }
        else if(FilementStatus[0] == 3)
        {
          if(LanguageRecbuf != 0)
          {
            // exchange to 23 page
            RTS_SndData(ExchangePageBase + 23, ExchangepageAddr);
          }
          else
          {
            RTS_SndData(ExchangePageBase + 65, ExchangepageAddr);
          }
        }
      }
      else if(recdat.data[0] == 0)
      {
        if(FilementStatus[0] == 1)
        {
          if(LanguageRecbuf != 0)
          {
            RTS_SndData(ExchangePageBase + 2, ExchangepageAddr);
          }
          else
          {
            RTS_SndData(ExchangePageBase + 46, ExchangepageAddr); 
          }
          PrinterStatusKey[0] = 0;
        }
        else if(FilementStatus[0] == 2)
        {
          // like the pause
          if(LanguageRecbuf != 0)
          {
            RTS_SndData(ExchangePageBase + 12, ExchangepageAddr);
          }
          else
          {
            RTS_SndData(ExchangePageBase + 54, ExchangepageAddr);
          }
        }
        else if(FilementStatus[0] == 3)
        {
          if(LanguageRecbuf != 0)
          {
            RTS_SndData(ExchangePageBase + 23, ExchangepageAddr);
          }
          else
          {
            RTS_SndData(ExchangePageBase + 65, ExchangepageAddr); 
          }
        }
        // recover the status waiting to check filements
        FilementStatus[0]  =  0;
      }
      break;
	  

    case PwrOffNoF:
      // Yes:continue to print the 3Dmode during power-off.
      if(recdat.data[0] == 1)
      {
		if(power_off_commands_count > 0) 
        {
		  if(LanguageRecbuf != 0)
          {
            // 1 for Heating
            RTS_SndData(1, IconPrintstatus);
            // exchange to 10 page
            RTS_SndData(ExchangePageBase + 10, ExchangepageAddr);
          }
          else
          {
            RTS_SndData(1 + CEIconGrap, IconPrintstatus);
            RTS_SndData(ExchangePageBase + 52, ExchangepageAddr); 
          }
		  recovery.resume();
		  power_off_type_yes = 1;
          FanStatus = false;
          PrintStatue[1] = 0;
          PrinterStatusKey[0] = 1;
          PrinterStatusKey[1] = 3;
          PoweroffContinue = true;
          TPShowStatus = InforShowStatus = true;
          // open the key of  checking card in  printing
          CardCheckStatus[0] = 1;
          PrintModeTime = Update_Time_Value = 0;
		}
      }
      else if(recdat.data[0] == 2)
      {
        InforShowStatus = true;
        TPShowStatus = false;
        Update_Time_Value = RTS_UPDATE_VALUE;
        if(LanguageRecbuf != 0)
        {
          // exchange to 1 page
          RTS_SndData(ExchangePageBase + 1, ExchangepageAddr);
        }
        else
        {
          // exchange to 45 page
          RTS_SndData(ExchangePageBase + 45, ExchangepageAddr);
        }

        card.stopSDPrint();
        queue.clear();
        quickstop_stepper();
        print_job_timer.stop();
        thermalManager.disable_all_heaters();
        print_job_timer.reset();
		recovery.purge();
		#if ENABLED(SDSUPPORT) && ENABLED(POWER_LOSS_RECOVERY)
          card.openJobRecoveryFile(true);
		  power_off_commands_count = 0;
        #endif
        wait_for_heatup = false;
        PrinterStatusKey[0] = 0;
		G29_status = 0;
        // for system
        delay(500);
      }
      break;

    case Volume:
      if(recdat.data[0] < 0) VolumeSet = 0;
      else if(recdat.data[0] > 255 ) VolumeSet = 0xFF;
      else VolumeSet = recdat.data[0];

      if(VolumeSet == 0)
      {
        RTS_SndData(0, VolumeIcon);
        RTS_SndData(9, SoundIcon);
      }
      else
      {
        RTS_SndData((VolumeSet+1)/32 - 1, VolumeIcon);
        RTS_SndData(8, SoundIcon);
      }
      eeprom_write_byte((unsigned char*)FONT_EEPROM+4, VolumeSet);
      RTS_SndData(VolumeSet<<8, SoundAddr+1);
      break;

    case Filename :
      if(card.isMounted() && (recdat.addr == FilenameChs))
      {
        if(recdat.data[0] > CardRecbuf.Filesum) break;

        CardRecbuf.recordcount = recdat.data[0] - 1;
        for(int j = 0;j < 10;j++)
        {
          RTS_SndData(0, Choosefilename + j);
        }
        int filelen = strlen(CardRecbuf.Cardshowfilename[CardRecbuf.recordcount]);
        filelen = (TEXTBYTELEN - filelen)/2;
        if(filelen > 0)
        {
          char buf[20];
          memset(buf,0,sizeof(buf));
          strncpy(buf,"         ",filelen);
          strcpy(&buf[filelen],CardRecbuf.Cardshowfilename[CardRecbuf.recordcount]);
          RTS_SndData(buf, Choosefilename);
        }
        else
        {
          RTS_SndData(CardRecbuf.Cardshowfilename[CardRecbuf.recordcount], Choosefilename);
        }

        for(int j = 0;j < 8;j++)
        {
          RTS_SndData(0,FilenameCount+j);
        }
        char buf[20];
        memset(buf,0,sizeof(buf));
        sprintf(buf,"%d/%d",(int)recdat.data[0], CardRecbuf.Filesum);
        RTS_SndData(buf, FilenameCount);
        delay(2);
        for(int j = 1;j <= CardRecbuf.Filesum;j++)
        {
          RTS_SndData((unsigned long)0xFFFF,FilenameNature + j*16);
          RTS_SndData(10,FilenameIcon1+j);
        }
        RTS_SndData((unsigned long)0x87F0,FilenameNature + recdat.data[0]*16);
        RTS_SndData(6,FilenameIcon1 + recdat.data[0]);
      }
      else if(recdat.addr == FilenamePlay)
      {
        // for sure
        if(recdat.data[0] == 1 && card.isMounted())
        {
          if(CardRecbuf.recordcount < 0)
          {
            break;
		  }

          char cmd[30];
          char* c;
          sprintf_P(cmd, PSTR("M23 %s"), CardRecbuf.Cardfilename[CardRecbuf.recordcount]);
          for (c = &cmd[4]; *c; c++) *c = tolower(*c);

          FilenamesCount = CardRecbuf.recordcount;
          memset(cmdbuf,0,sizeof(cmdbuf));
          strcpy(cmdbuf,cmd);

          #if CHECKFILEMENT
            //checking filement status during printing beginning 
            if(RTS_CheckFilement(1)) break;
          #endif

          // InforShowoStatus = true;
          queue.enqueue_one_now(cmd);
          queue.enqueue_now_P(PSTR("M24"));
          // clean screen.
          for(int j = 0;j < 10;j++)
          {
            RTS_SndData(0,Printfilename+j);
          }

          int filelen = strlen(CardRecbuf.Cardshowfilename[CardRecbuf.recordcount]);
          filelen = (TEXTBYTELEN - filelen)/2;
          if(filelen > 0)
          {
            char buf[20];
            memset(buf,0,sizeof(buf));
            strncpy(buf,"         ",filelen);
            strcpy(&buf[filelen],CardRecbuf.Cardshowfilename[CardRecbuf.recordcount]);
            RTS_SndData(buf, Printfilename);
          }
          else
          {
            RTS_SndData(CardRecbuf.Cardshowfilename[CardRecbuf.recordcount], Printfilename);
          }
          delay(2);
          const uint8_t p = parser.byteval('P');
		  thermalManager.set_fan_speed(p, 255);
          FanStatus = false;

          if(LanguageRecbuf != 0)
          {
            // 1 for Heating 
            RTS_SndData(1,IconPrintstatus);
            delay(2);
            // exchange to 10 page
            RTS_SndData(ExchangePageBase + 10, ExchangepageAddr);
          }
          else
          {
            // 1 for Heating 
            RTS_SndData(1+CEIconGrap,IconPrintstatus);
            delay(2);
            RTS_SndData(ExchangePageBase + 52, ExchangepageAddr); 
          }
          TPShowStatus = InforShowStatus = true;
          PrintModeChange = true;
          PrintStatue[1] = 0;
          // close the checking filements status in printing
          FilementStatus[1] = 0;
          PrinterStatusKey[0] = 1;
          PrinterStatusKey[1] = 3;
          // open the key of  checking card in  printing
          CardCheckStatus[0] = 1;
          PrintModeTime = 1;
          Update_Time_Value = 0;
        }
        else if(recdat.data[0] == 0)	// return to main page
        {
          InforShowStatus = true;
          TPShowStatus = false;
        }
      }
      break;
    case Advanced:
	  if(recdat.addr == PLANozzle)
	  {   
		ui.preheat_hotend_temp[0] = recdat.data[0];
		settings.save();
	  }
	  else if(recdat.addr == PLABed)
	  {
		ui.preheat_bed_temp[0] = recdat.data[0];
		settings.save();		
	  }
	  else if(recdat.addr == ABSNozzle)
	  {
		ui.preheat_hotend_temp[1] = recdat.data[0];
		settings.save();		
	  }
	  else if(recdat.addr == ABSBed)
	  {
		ui.preheat_bed_temp[1] = recdat.data[0];
		settings.save();		
	  }
	  else if(recdat.addr == VmaxX)
	  {
		planner.settings.max_feedrate_mm_s[X_AXIS] = recdat.data[0];
		settings.save();

	  }
	  else if(recdat.addr == VmaxY)
	  {
		planner.settings.max_feedrate_mm_s[Y_AXIS] = recdat.data[0];
		settings.save();
	  }
	  else if(recdat.addr == VmaxZ)
	  {
		planner.settings.max_feedrate_mm_s[Z_AXIS] = recdat.data[0];
		settings.save();
	  }
	  else if(recdat.addr == VmaxE)
	  {
		planner.settings.max_feedrate_mm_s[E_AXIS] = recdat.data[0];
		settings.save();
	  }
	  else if(recdat.addr == Accel)
	  {
		planner.settings.acceleration = recdat.data[0];
		settings.save();
	  }
	  else if(recdat.addr == ARetract)
	  {
		planner.settings.retract_acceleration = recdat.data[0];
		settings.save();
	  }
	  else if(recdat.addr == ATravel)
	  {
		planner.settings.travel_acceleration = recdat.data[0];
		settings.save();
	  }
	  else if(recdat.addr == AmaxX)
	  {
		planner.settings.max_acceleration_mm_per_s2[A_AXIS] = recdat.data[0];
		settings.save();
	  }
	  else if(recdat.addr == AmaxY)
	  {
		planner.settings.max_acceleration_mm_per_s2[B_AXIS] = recdat.data[0];
		settings.save();
	  }
	  else if(recdat.addr == AmaxZ)
	  {
		planner.settings.max_acceleration_mm_per_s2[C_AXIS] = recdat.data[0];
		settings.save();
	  }
	  else if(recdat.addr == AmaxE)
	  {
		planner.settings.max_acceleration_mm_per_s2[E_AXIS] = recdat.data[0];
		settings.save();
	  }
	  else if(recdat.addr == Xjerk)
	  {
		planner.max_jerk[X_AXIS] = ((float)recdat.data[0])/10;
		settings.save();
	  }
	  else if(recdat.addr == Yjerk)
	  {
		planner.max_jerk[Y_AXIS] = ((float)recdat.data[0])/10;
		settings.save();
	  }
	  else if(recdat.addr == Zjerk)
	  {
		planner.max_jerk[Z_AXIS] = ((float)recdat.data[0])/10;
		settings.save();
	  }
	  else if(recdat.addr == Ejerk)
	  {
		planner.max_jerk[E_AXIS] = ((float)recdat.data[0])/10;
		settings.save();
	  }
	  else if(recdat.addr == Xsteps)
	  {
		planner.settings.axis_steps_per_mm[A_AXIS] = ((float)recdat.data[0])/10;
		settings.save();
	  }
	  else if(recdat.addr == Ysteps)
	  {
		planner.settings.axis_steps_per_mm[B_AXIS] = ((float)recdat.data[0])/10;
		settings.save();
	  }
	  else if(recdat.addr == Zsteps)
	  {
		planner.settings.axis_steps_per_mm[C_AXIS] = ((float)recdat.data[0])/10;
		settings.save();
	  }
	  else if(recdat.addr == Esteps)
	  {
		planner.settings.axis_steps_per_mm[E_AXIS_N(active_extruder)] = ((float)recdat.data[0])/10;
		settings.save();
	  }
      break;
    default:
      break;
	  
  }
  memset(&recdat,0 , sizeof(recdat));
  recdat.head[0] = FHONE;
  recdat.head[1] = FHTWO;
}
void EachMomentUpdate()
{
  millis_t ms = millis();
  if(ms > next_rts_update_ms && InforShowStatus)
  {
    if ((power_off_type_yes == 0)  && LCD_sd_status && power_off_commands_count > 0)
	{
      if(startprogress == 0)
      {
        rtscheck.RTS_SndData(StartSoundSet, SoundAddr);

        if(VolumeSet == 0)
        {
          rtscheck.RTS_SndData(0, VolumeIcon);
          rtscheck.RTS_SndData(9, SoundIcon);
        }
        else
        {
          rtscheck.RTS_SndData((VolumeSet+1)/32 - 1, VolumeIcon);
          rtscheck.RTS_SndData(8, SoundIcon);
        }
        rtscheck.RTS_SndData(VolumeSet, VolumeIcon-2);
        rtscheck.RTS_SndData(VolumeSet<<8, SoundAddr+1);
      }
      if(startprogress <= 100)
      {
        rtscheck.RTS_SndData(startprogress,StartIcon);
      }
      delay(30);
      if((startprogress +=1) > 100)
      {
        power_off_type_yes = 1;
        for (uint16_t i = 0; i < CardRecbuf.Filesum ; i++) 
        {
          if(!strcmp(CardRecbuf.Cardfilename[i], &recovery.info.sd_filename[1]))
          {
            InforShowStatus = true;
            int filelen = strlen(CardRecbuf.Cardshowfilename[i]);
            filelen = (TEXTBYTELEN - filelen)/2;
            if(filelen > 0)
            {
              char buf[20];
              memset(buf,0,sizeof(buf));
              strncpy(buf,"         ",filelen);
              strcpy(&buf[filelen],CardRecbuf.Cardshowfilename[i]);
              rtscheck.RTS_SndData(buf, Printfilename);
            }
            else
            {
              // filenames
              rtscheck.RTS_SndData(CardRecbuf.Cardshowfilename[i],Printfilename);
            }
            if(LanguageRecbuf != 0)
            {
              // exchange to 36 page
              rtscheck.RTS_SndData(ExchangePageBase + 36, ExchangepageAddr);
            }
            else
            {
              rtscheck.RTS_SndData(ExchangePageBase + 76, ExchangepageAddr);
            }
            break;
          }
        }
      }
	  return;
    }
	else if((power_off_type_yes == 0) && !power_off_commands_count)
	{
		if(startprogress == 0)
		{
		  rtscheck.RTS_SndData(StartSoundSet, SoundAddr);

		  if(VolumeSet == 0)
		  {
			rtscheck.RTS_SndData(0, VolumeIcon);
			rtscheck.RTS_SndData(9, SoundIcon);
		  }
		  else
		  {
			rtscheck.RTS_SndData((VolumeSet+1)/32 - 1, VolumeIcon);
			rtscheck.RTS_SndData(8, SoundIcon);
		  }
		  rtscheck.RTS_SndData(VolumeSet, VolumeIcon-2);
		  rtscheck.RTS_SndData(VolumeSet<<8, SoundAddr+1);
		}
		if(startprogress <= 100)
		{
		  rtscheck.RTS_SndData(startprogress,StartIcon);
		}
		delay(30);
		if((startprogress +=1) > 100)
		{
		  power_off_type_yes = 1;
		  InforShowStatus = true;
		  TPShowStatus = false;
		  Update_Time_Value = RTS_UPDATE_VALUE;
		  if(LanguageRecbuf != 0)
		  {
			// exchange to 1 page
			rtscheck.RTS_SndData(ExchangePageBase + 1, ExchangepageAddr);
		  }
		  else
		  {
			rtscheck.RTS_SndData(ExchangePageBase + 45, ExchangepageAddr);
		  }
		}
		  return;
	}
    else
    {
      // need to optimize
      if (TPShowStatus && ms != 0)
      {
        duration_t elapsed = print_job_timer.duration();
        static unsigned int last_cardpercentValue = 100; 
        rtscheck.RTS_SndData(elapsed.value/3600,Timehour);		
        rtscheck.RTS_SndData((elapsed.value%3600)/60,Timemin);	

        if(IS_SD_PRINTING() && last_cardpercentValue != card.percentDone())
        {
          if((unsigned int) card.percentDone() > 0)
          {
            Percentrecord = card.percentDone() + 1;
            if(Percentrecord <= 100)
            {
              rtscheck.RTS_SndData((unsigned int)Percentrecord, PrintscheduleIcon);
            }
            else
            {
              rtscheck.RTS_SndData(100 ,PrintscheduleIcon);
            }
          }	
          else
          {
            rtscheck.RTS_SndData(0,PrintscheduleIcon);
          }
          rtscheck.RTS_SndData((unsigned int) card.percentDone(),Percentage);
          last_cardpercentValue = card.percentDone();
        }

        // save z offset
        if(probe_offset.z != last_zoffset)
        {
          settings.save();
          last_zoffset = probe_offset.z;
        } 
      }

      // float temp_buf = thermalManager.temp_hotend[0].celsius;
      rtscheck.RTS_SndData(thermalManager.temp_hotend[0].celsius, NozzleTemp);
      rtscheck.RTS_SndData(thermalManager.temp_bed.celsius, Bedtemp);
	  rtscheck.RTS_SndData(current_position.z*10, Zaxis);
      if(last_temp_bed != thermalManager.temp_bed.target || (last_target_temperature[0] != thermalManager.temp_hotend[0].target))
      {
        thermalManager.setTargetHotend(thermalManager.temp_hotend[0].target, 0);
        thermalManager.setTargetBed(thermalManager.temp_bed.target);
        rtscheck.RTS_SndData(thermalManager.temp_hotend[0].target, NozzlePreheat);
        rtscheck.RTS_SndData(thermalManager.temp_bed.target, BedPreheat);

        if(IS_SD_PRINTING())
        {
          // keep the icon
        }
        else if(last_temp_bed < thermalManager.temp_bed.target || (last_target_temperature[0] < thermalManager.temp_hotend[0].target))
        {
          if(LanguageRecbuf != 0)
          {
            // 1 for Heating
            rtscheck.RTS_SndData(1, IconPrintstatus);
          }
          else
          {
            rtscheck.RTS_SndData(1 + CEIconGrap, IconPrintstatus);
          }
          Update_Time_Value = 0;
          PrinterStatusKey[1] =( PrinterStatusKey[1] == 0? 1 : PrinterStatusKey[1]);
        }
        else if(last_temp_bed > thermalManager.temp_bed.target || (last_target_temperature[0] > thermalManager.temp_hotend[0].target))
        {
          if(LanguageRecbuf != 0)
          {
            // 8 for Cooling
            rtscheck.RTS_SndData(8,IconPrintstatus);
          }
          else
          {
            rtscheck.RTS_SndData(8+CEIconGrap,IconPrintstatus);
          }
          Update_Time_Value = 0;
          PrinterStatusKey[1] =( PrinterStatusKey[1] == 0? 2 : PrinterStatusKey[1] );
        }
        last_temp_bed = thermalManager.temp_bed.target;
        last_target_temperature[0] = thermalManager.temp_hotend[0].target;
      }

      // statuse of loadfilement and unloadfinement when temperature is less than
      if(NozzleTempStatus[0] || NozzleTempStatus[2])
      {
        unsigned int IconTemp;
        if(thermalManager.temp_hotend[0].target)
          IconTemp = thermalManager.temp_hotend[0].celsius * 100/thermalManager.temp_hotend[0].target;
        else 
          IconTemp = 100;

        if(IconTemp >= 100)
        {
          IconTemp = 100;
        }
        rtscheck.RTS_SndData(IconTemp, HeatPercentIcon);

        if(thermalManager.temp_hotend[0].celsius >= thermalManager.temp_hotend[0].target && NozzleTempStatus[0])
        {
          NozzleTempStatus[1] = 0;
          NozzleTempStatus[0] = 0;
          rtscheck.RTS_SndData(10*ChangeMaterialbuf[0], FilementUnit1);	
          rtscheck.RTS_SndData(10*ChangeMaterialbuf[1], FilementUnit2);
          if(LanguageRecbuf != 0)
          {
            // exchange to 23 page
            rtscheck.RTS_SndData(ExchangePageBase + 23, ExchangepageAddr);
          }
          else
          {
            rtscheck.RTS_SndData(ExchangePageBase + 65, ExchangepageAddr);
          }
          RTS_line_to_current(E_AXIS);
          active_extruder = original_extruder;
          // delay(current_position[E_AXIS] * 1000);
        }
        else if(thermalManager.temp_hotend[0].celsius >= thermalManager.temp_hotend[0].target && NozzleTempStatus[2])
        {
          // SERIAL_ECHOPAIR("\n ***NozzleTempStatus[2] =",(int)NozzleTempStatus[2]);
          startprogress = NozzleTempStatus[2] = 0;
          TPShowStatus = true;
          rtscheck.RTS_SndData(4, ExchFlmntIcon);
          if(LanguageRecbuf != 0)
          {
            rtscheck.RTS_SndData(ExchangePageBase + 8, ExchangepageAddr);
          }
          else
          {
            rtscheck.RTS_SndData(ExchangePageBase + 83, ExchangepageAddr);
          }
        }
        else if( NozzleTempStatus[2])
        {
          rtscheck.RTS_SndData((startprogress++)%5, ExchFlmntIcon);
        }
      }
      if(AutohomeKey)
      {
        rtscheck.RTS_SndData(AutoHomeIconNum++,AutoZeroIcon);
        if(AutoHomeIconNum > 9)	AutoHomeIconNum = 0;
      }
    }
    next_rts_update_ms = ms + RTS_UPDATE_INTERVAL + Update_Time_Value;
  }
  
}

// looping at the loop function
void RTSUpdate()
{
  //Check the status of card 
  rtscheck.RTS_SDCardUpate();

  #if CHECKFILEMENT
    // checking filement status during printing
    if(FilementStatus[1] == 2 && true == IS_SD_PRINTING())
    {
      char cmd[2][30];
      if((0 == READ(CHECK_MATWEIAL0)) || (0 == READ(CHECK_MATWEIAL0)))
      {
        Checkfilenum++;
        delay(5);
        if(Checkfilenum>50)
        {
          if(LanguageRecbuf != 0)
            rtscheck.RTS_SndData(ExchangePageBase + 86, ExchangepageAddr);	
          else
            rtscheck.RTS_SndData(ExchangePageBase + 87, ExchangepageAddr); 
          waitway = 5;
          pause_z = current_position[Z_AXIS];
          pause_e = current_position[E_AXIS] - 10;
          card.pauseSDPrint();
          print_job_timer.pause();
		  SERIAL_ECHOLN("===......===");
          if(!temphot)
            temphot = thermalManager.degTargetHotend(0);
          if(!tempbed)
            tempbed = thermalManager.degTargetBed();
          // thermalManager.setTargetHotend(0, 0);
          thermalManager.setTargetBed(0);

          // for returning the corresponding page
          PrintStatue[1] = 1;
          // no filements during printing
          // FilementStatus[0] = 2;
          Checkfilenum = 0;
          FilementStatus[1] = 0;
          PrinterStatusKey[1] = 4;

          TPShowStatus = false;
          Update_Time_Value = 0;
          // Z axis cancel to improve 5mm
          queue.inject_P(PSTR("G28 X0 Y0"));
        }
      }
    }
  #endif

  EachMomentUpdate();

  // wait to receive massage and response
  if(rtscheck.RTS_RecData() > 0)
  {
    rtscheck.RTS_HandleData();
  }
}
