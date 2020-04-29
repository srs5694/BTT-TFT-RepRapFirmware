#include "includes.h"
#include "parseACK.h"

char dmaL2Cache[ACK_MAX_SIZE];
static u16 ack_index=0;
static u8 ack_cur_src = SERIAL_PORT;
int MODEselect;
// Ignore reply "echo:" message (don't display in popup menu)
const char *const ignoreEcho[] = {
  "busy: processing",
  "Now fresh file:",
  "Probe Z Offset:",
  "Flow:",
  "echo:;",
  "echo:  G",
  "echo:  M",
};

bool portSeen[_UART_CNT] = {false, false, false, false, false, false};

void setCurrentAckSrc(uint8_t src)
{
  ack_cur_src = src;
  portSeen[src] = true;
}

static char ack_seen(const char *str)
{
  u16 i;
  for(ack_index=0; ack_index<ACK_MAX_SIZE && dmaL2Cache[ack_index]!=0; ack_index++)
  {
    for(i=0; str[i]!=0 && dmaL2Cache[ack_index+i]!=0 && dmaL2Cache[ack_index+i]==str[i]; i++)
    {}
    if(str[i]==0)
    {
      ack_index += i;
      return true;
    }
  }
  return false;
}
static char ack_cmp(const char *str)
{
  u16 i;
  for(i=0; i<ACK_MAX_SIZE && str[i]!=0 && dmaL2Cache[i]!=0; i++)
  {
    if(str[i] != dmaL2Cache[i])
    return false;
  }
  if(dmaL2Cache[i] != 0) return false;
  return true;
}


static float ack_value()
{
  return (strtod(&dmaL2Cache[ack_index], NULL));
}

// Read the value after the , if exists
static float ack_second_value()
{
  char *secondValue = strchr(&dmaL2Cache[ack_index],',');
  if(secondValue != NULL)
  {
    return (strtod(secondValue+1, NULL));
  }
  else
  {
    return -0.5;
  }
}

static float ack_third_value()
{
  char *secondValue = strchr(&dmaL2Cache[ack_index],','); 
  secondValue = strchr(secondValue+1,','); //второе вхождение
  if(secondValue != NULL)
  {
    return (strtod(secondValue+1, NULL));
  }
  else
  {
    return -0.5;
  }
}

void ackPopupInfo(const char *info)
{
  if(infoMenu.menu[infoMenu.cur] == menuParameterSettings) return;

  char *t = strtok(&dmaL2Cache[ack_index],"\"");

  if (info == echomagic)
  {
    statusScreen_setMsg((u8*)info,(u8*)t);
  }
  if (infoMenu.menu[infoMenu.cur] == menuTerminal) return;
  if (infoMenu.menu[infoMenu.cur] == menuStatus && info == echomagic) return;

  popupReminder((u8*)info,(u8*)t);
}

bool dmaL1NotEmpty(uint8_t port)
{
  return dmaL1Data[port].rIndex != dmaL1Data[port].wIndex;
}

void syncL2CacheFromL1(uint8_t port)
{
  uint16_t i = 0;
  for (i = 0; dmaL1NotEmpty(port) && dmaL2Cache[i-1] != '\n'; i++)
  {
    dmaL2Cache[i] = dmaL1Data[port].cache[dmaL1Data[port].rIndex];
    dmaL1Data[port].rIndex = (dmaL1Data[port].rIndex + 1) % DMA_TRANS_LEN;
  }
  dmaL2Cache[i] = 0; // End character
}

void parseACK(void)
{
  bool avoid_terminal = false;
  if(infoHost.rx_ok[SERIAL_PORT] != true) return; //not get response data

  while(dmaL1NotEmpty(SERIAL_PORT))
  {
    syncL2CacheFromL1(SERIAL_PORT);
    infoHost.rx_ok[SERIAL_PORT] = false;

    if(infoHost.connected == false) //not connected to Marlin
    {
      // if(!ack_seen("beep_freq") || !ack_seen("message") || !ack_seen("status"))  goto parse_end;  //the first response should be such as "T:25/50 ok\n"
      updateNextHeatCheckTime();
      infoHost.connected = true;
      storeCmd("M409 K\"network.interfaces[0].actualIP\"\n"); //RRF3
      // storeCmd("M115\n");
      // storeCmd("M503 S0\n");
      // storeCmd("M92\n"); // Steps/mm of extruder is an important parameter for Smart filament runout
                           // Avoid can't getting this parameter due to disabled M503 in Marlin
    }

    // Gcode command response
    if(requestCommandInfo.inWaitResponse && ack_seen(requestCommandInfo.startMagic))
    {
      requestCommandInfo.inResponse = true;
      requestCommandInfo.inWaitResponse = false;
    }
    if(requestCommandInfo.inResponse)
    {
      if(strlen(requestCommandInfo.cmd_rev_buf)+strlen(dmaL2Cache) < CMD_MAX_REV)
      {
        strcat(requestCommandInfo.cmd_rev_buf, dmaL2Cache);

        if(ack_seen(requestCommandInfo.errorMagic ))
        {
          requestCommandInfo.done = true;
          requestCommandInfo.inResponse = false;
          requestCommandInfo.inError = true;
        }
        else if(ack_seen(requestCommandInfo.stopMagic ))
        {
          requestCommandInfo.done = true;
          requestCommandInfo.inResponse = false;
        }
      }
      else
      {
        requestCommandInfo.done = true;
        requestCommandInfo.inResponse = false;
        ackPopupInfo(errormagic);
      }
      infoHost.wait = false;
      goto parse_end;
    }
    // end

    if(ack_seen("{\"status\""))
    {
      infoHost.wait = false;
      avoid_terminal = infoSettings.terminalACK;

      if(ack_seen("axesHomed\":["))
      {
        if (ack_value() != 0 && ack_second_value() != 0 && ack_third_value() != 0) 
        {
          coordinateSetKnown(true);
        }
        else 
        {
          coordinateSetKnown(false);
        }
      }

      if(ack_seen("xyz\":["))
      {
        storegantry(0, ack_value());
        storegantry(1, ack_second_value());
        storegantry(2, ack_third_value());
      }

      // parse Extruder
      if(ack_seen("extr\":"))
      {
        coordinateSetAxisActualSteps(E_AXIS, ack_value());
      }

      if (ack_seen("fanPercent\":["))
      {
        fanSetSpeed(0, ack_value()*2.55+0.5);
        // fanSetSpeed(1, ack_second_value());
      }

      // parse and store feed rate percentage
      if(ack_seen("speedFactor\":"))
      {
        speedSetPercent(0,ack_value());
      }

      // parse and store flow rate percentage
      if(ack_seen("extrFactors\":["))
      {
        speedSetPercent(1,ack_value());
      }

      if(ack_seen("babystep\":"))
      {
        setBabyStep(ack_value());
      }

      //parse temperature
      if(ack_seen("current\":["))
      {
        // TOOL i = heatGetCurrentToolNozzle();
        TOOL i = 0;
        heatSetCurrentTemp(i, ack_value()+0.5);

        // TOOL i = heatGetCurrentToolNozzle();
        TOOL ii = 1;
        heatSetCurrentTemp(ii, ack_second_value()+0.5);

        if(ack_seen(",\"active\":[") && !heatGetSendWaiting(i)) {
          heatSyncTargetTemp(i, ack_value()+0.5);
        }

        if(ack_seen("{\"active\":[[") && !heatGetSendWaiting(ii)) {
          heatSyncTargetTemp(ii, ack_value()+0.5);
        }
        
        updateNextHeatCheckTime();
      }
      
    }
    else if(ack_seen("message\":\""))
    {
      BUZZER_PLAY(sound_notify);
      ackPopupInfo(echomagic);
    }


    // if(ack_seen("Count E:")) // Parse actual extruder position, response of "M114 E\n", required "M114_DETAIL" in Marlin
    // {
    //   coordinateSetAxisActualSteps(E_AXIS, ack_value());
    // }
    if(infoHost.printing && ack_seen("status\":\"I"))
    {
      infoHost.printing = false;
      completePrinting();
    }
    // busy (e.g. running a macro)
    // этот статус не отдаёт
    // else if(infoHost.printing && ack_seen("status\":\"B"))
    // {
    //   reminderMessage(LABEL_BUSY, STATUS_BUSY);
    // }
    // on PAUSE
    else if(infoHost.printing && ack_seen("status\":\"A"))
    {
      setPause(true);
    }
    else if(ack_seen("status\":\"P"))
    {
      if (isPause()) setPause(false);

      setPrinting(true);

      if(infoMenu.menu[infoMenu.cur] != menuPrinting && !infoHost.printing) {
        infoMenu.menu[++infoMenu.cur] = menuPrinting;
        infoHost.printing=true;
        storeCmd("M409 K\"job.file\"\n");
      }

      if(ack_seen("fraction_printed\":"))
      {
        setPrintCur(ack_value()*100);
      }

      

    }

    // ответ от M409 K"network.interfaces[0]"
    else if(ack_seen("network.interfaces[0].actualIP") && ack_seen("result\":\""))
    {
      char *t = strtok(&dmaL2Cache[ack_index],"\"");
      if (strcmp(t, "0.0.0.0") ==0)
      {
        statusScreen_setMsg((u8*)echomagic,(u8*)"Connecting...");
        storeCmd("M409 K\"network.interfaces[0].actualIP\"\n"); //  перезапрашиваем до установления связи
      }
      else
      {
        BUZZER_PLAY(sound_notify);
        ackPopupInfo(echomagic);
      }
    }
    // ответ от M409 K"job.file"
    else if(ack_seen("job.file") && ack_seen("fileName\":\""))
    {
      // BUZZER_PLAY(sound_notify);
      // ackPopupInfo(echomagic);
      // infoFile.title = strtok ((u8 *)dmaL2Cache + ack_index ,"\"");
      // printingItems.title.address = getCurGcodeName(strtok ((u8 *)dmaL2Cache + ack_index ,"\""));

    }
    // beep buzzer
    #ifdef BUZZER_PIN
    else if(ack_seen("beep_freq\":"))
    {
      uint32_t freq = ack_value();
      if(ack_seen("beep_length\":"))
      {
        Buzzer_TurnOn(freq,ack_value());
      }
    }
     #endif

  parse_end:
    if(ack_cur_src != SERIAL_PORT)
    {
      Serial_Puts(ack_cur_src, dmaL2Cache);
    }
    else if (!ack_seen("ok"))
    {
      // make sure we pass on spontaneous messages to all connected ports (since these can come unrequested)
      for (int port = 0; port < _UART_CNT; port++)
      {
        if (port != SERIAL_PORT && portSeen[port])
        {
          // pass on this one to anyone else who might be listening
          Serial_Puts(port, dmaL2Cache);
        }
      }
    }

    if (avoid_terminal != true){
      sendGcodeTerminalCache(dmaL2Cache, TERMINAL_ACK);
    }
  }
}

void parseRcvGcode(void)
{
  #ifdef SERIAL_PORT_2
    uint8_t i = 0;
    for(i = 0; i < _UART_CNT; i++)
    {
      if(i != SERIAL_PORT && infoHost.rx_ok[i] == true)
      {
        infoHost.rx_ok[i] = false;
        while(dmaL1NotEmpty(i))
        {
          syncL2CacheFromL1(i);
          storeCmdFromUART(i, dmaL2Cache);
        }
      }
    }
  #endif
}
