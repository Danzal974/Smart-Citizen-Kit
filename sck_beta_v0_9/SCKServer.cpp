/*

  SCKServer.cpp

*/


#include "Constants.h"
#include "SCKServer.h"
#include "SCKBase.h"
#include "SCKAmbient.h"
#include <Wire.h>
#include <EEPROM.h>

#define debugServer  false

SCKBase base__;
SCKServer server__;
SCKAmbient ambient__;

#define TIME_BUFFER_SIZE 20

boolean SCKServer::time(char *time_) {
  boolean ok = false;
  uint8_t count = 0;
  byte retry = 0;

  byte hosttime = 0;
  while ((retry < 5) && (!ok)) 
  {
    retry++;
    if (base__.enterCommandMode())
    {
      if (base__.open(HOSTADDR[hosttime], 80))
      {
        Serial1.print((char *)"GET ");
        Serial1.print(TIMEENDPOINT[hosttime]);
        Serial1.print(WEB[0]);
        Serial1.print(HOSTADDR[hosttime]);
        Serial1.print(WEB[1]);
        Serial1.print("\n");

        if (base__.findInResponse("UTC:", 2000))
        {
          char newChar;
          byte offset = 0;
          unsigned long time = millis();
          while (offset < TIME_BUFFER_SIZE) {
            if (Serial1.available())
            {
              newChar = Serial1.read();
              time = millis();
              if (newChar == '#') {
                ok = true;
                time_[offset] = '\x00';
                break;
              }
              else if (newChar != -1) {
                if (newChar == ',')
                {
                  if (count < 2) time_[offset] = '-';
                  else if (count > 2) time_[offset] = ':';
                  else time_[offset] = ' ';
                  count++;
                }
                else time_[offset] = newChar;
                offset++;
              }
            }
            else if ((millis() - time) > 1000)
            {
              ok = false;
              break;
            }
          }
        }
        else {
#if debugServer
          Serial.println("FAIL:(");
#endif
        }
        base__.close();
      }
      //}
    }
  }
  if (!ok)
  {
    time_[0] = '#';
    time_[1] = 0x00;
  }
  base__.exitCommandMode();
  return ok;
}

boolean SCKServer::RTCupdate(char *time_) {
  byte retry = 0;
  if (base__.checkRTC()) {
    if (time(time_)) {
      while (retry < 5) {
        retry++;
        if (base__.RTCadjust(time_)) {
          return true;
        }
      }
    }
  }
  return false;
}

void SCKServer::json_update(uint16_t updates, long *value, char *time, boolean isMultipart)
{
#if debugServer
  Serial.print(F("["));
#endif
  Serial1.print(F("["));
  for (int i = 0; i < updates; i++)
  {
    readFIFO();
    if ((i < (updates - 1)) || (isMultipart))
    {
      Serial1.print(F(","));
#if debugServer
      Serial.print(F(","));
#endif
    }
  }

  if (isMultipart)
  {
    byte i;
    for (i = 0; i < 9; i++)
    {
      Serial1.print(SERVER[i]);
      Serial1.print(value[i]);
    }
    Serial1.print(SERVER[i]);
    Serial1.print(time);
    Serial1.print(SERVER[i + 1]);
    Serial1.println(F("]"));
    Serial1.println();

#if debugServer
    for (i = 0; i < 9; i++)
    {
      Serial.print(SERVER[i]);
      Serial.print(value[i]);
    }
    Serial.print(SERVER[i]);
    Serial.print(time);
    Serial.print(SERVER[i + 1]);
#endif
  }
  Serial1.println(F("]"));
  Serial1.println();
#if debugServer
  Serial.println(F("]"));
#endif
}

void SCKServer::addFIFO(long *value, char *time)
{
  uint16_t updates = (base__.readData(EE_ADDR_NUMBER_WRITE_MEASURE, INTERNAL) - base__.readData(EE_ADDR_NUMBER_READ_MEASURE, INTERNAL)) / ((SENSORS) * 4 + TIME_BUFFER_SIZE);
  if (updates < MAX_MEMORY)
  {
    int eeaddress = base__.readData(EE_ADDR_NUMBER_WRITE_MEASURE, INTERNAL);
    int i = 0;
    for (i = 0; i < 9; i++)
    {
      base__.writeData(eeaddress + i * 4, value[i], EXTERNAL);
    }
    base__.writeData(eeaddress + i * 4, 0, time, EXTERNAL);
    eeaddress = eeaddress + (SENSORS) * 4 + TIME_BUFFER_SIZE;
    base__.writeData(EE_ADDR_NUMBER_WRITE_MEASURE, eeaddress, INTERNAL);
  }
  else
  {
#if debugEnabled
    if (!ambient__.debug_state()) Serial.println(F("Memory limit exceeded!!"));
#endif
  }
}

void SCKServer::readFIFO()
{
  int i = 0;
  int eeaddress = base__.readData(EE_ADDR_NUMBER_READ_MEASURE, INTERNAL);
  for (i = 0; i < 9; i++)
  {
    Serial1.print(SERVER[i]);
    Serial1.print(base__.readData(eeaddress + i * 4, EXTERNAL)); //SENSORS
  }
  Serial1.print(SERVER[i]);
  Serial1.print(base__.readData(eeaddress + i * 4, 0, EXTERNAL)); //TIME
  Serial1.print(SERVER[i + 1]);

#if debugServer
  for (i = 0; i < 9; i++)
  {
    Serial.print(SERVER[i]);
    Serial.print(base__.readData(eeaddress + i * 4, EXTERNAL)); //SENSORS
  }
  Serial.print(SERVER[i]);
  Serial.print(base__.readData(eeaddress + i * 4, 0, EXTERNAL)); //TIME
  Serial.print(SERVER[i + 1]);
#endif

  eeaddress = eeaddress + (SENSORS) * 4 + TIME_BUFFER_SIZE;
  if (eeaddress == base__.readData(EE_ADDR_NUMBER_WRITE_MEASURE, INTERNAL))
  {
    base__.writeData(EE_ADDR_NUMBER_WRITE_MEASURE, 0, INTERNAL);
    base__.writeData(EE_ADDR_NUMBER_READ_MEASURE, 0, INTERNAL);
  }
  else base__.writeData(EE_ADDR_NUMBER_READ_MEASURE, eeaddress, INTERNAL);
}

#define numbers_retry 5

boolean SCKServer::update(long *value, char *time_)
{
  value[8] = base__.scan();  //Wifi Nets
  byte retry = 0;
  if (time(time_)) //Update server time
  {
    if (base__.checkRTC())
    {
      while (!base__.RTCadjust(time_) && (retry < numbers_retry)) retry++;
    }
  }
  else if (base__.checkRTC())
  {
    base__.RTCtime(time_);
#if debugEnabled
    if (!ambient__.debug_state()) Serial.println(F("Fail server time!!"));
#endif
  }
  else
  {
    time_ = "#";
    return false;
  }
  return true;
}

boolean SCKServer::connect(byte host)
{
  int retry = 0;

  while (true) {
    if (base__.open(HOSTADDR[host], 80)) break; 
    else
    {
      retry++;
      if (retry >= numbers_retry) return false;
    }
  }
  Serial1.print("PUT ");
  Serial1.print(ENDPTHTTP[host]); // in Constants 
  Serial1.print(WEB[0]);
  Serial1.print(HOSTADDR[host]); // 
  Serial1.print(WEB[1]);
  if (host == 1) Serial1.print(AUTHPH); // required for communecter.org
  Serial1.print(WEB[2]);
  Serial1.println(base__.readData(EE_ADDR_MAC, 0, INTERNAL)); //MAC ADDRESS
  Serial1.print(WEB[3]);
  Serial1.println(base__.readData(EE_ADDR_APIKEY, 0, INTERNAL)); //Apikey
  Serial1.print(WEB[4]);
  Serial1.println(FirmWare); //Firmware version
  Serial1.print(WEB[5]);
  return true;
}


void SCKServer::send(boolean sleep, boolean *wait_moment, long *value, char *time, boolean instant) {
  *wait_moment = true;
  if (base__.checkRTC()) base__.RTCtime(time);
  char tmpTime[19];
  strncpy(tmpTime, time, 20);

  uint16_t updates = (base__.readData(EE_ADDR_NUMBER_WRITE_MEASURE, INTERNAL) - base__.readData(EE_ADDR_NUMBER_READ_MEASURE, INTERNAL)) / ((SENSORS) * 4 + TIME_BUFFER_SIZE);
  uint16_t NumUpdates = base__.readData(EE_ADDR_NUMBER_UPDATES, INTERNAL); // Number of readings before batch update

boolean wificonnected=false;
for(byte j=0;j<nb_host;j++){
  

  if (updates >= (NumUpdates - 1) || instant)
  {
    if (sleep && j==0)
    {
#if debugEnabled
      if (!ambient__.debug_state()) Serial.println(F("SCK Waking up..."));
#endif
      digitalWrite(AWAKE, HIGH);
    }
    if (!wificonnected) wificonnected = base__.connect();  //Wifi connect
    if (wificonnected)
    {
#if debugEnabled
      if (!ambient__.debug_state()) Serial.println(F("SCK Connected to Wi-Fi!!"));
#endif
      if (update(value, time)) //Update time and nets
      {
#if debugEnabled
        if (!ambient__.debug_state())
        {
          Serial.print(F("updates = "));
          Serial.println(updates + 1);
        }
#endif
        int num_post = updates;
        int cycles = cycles = updates / POST_MAX;;
        if (updates > POST_MAX)
        {
          for (int i = 0; i < cycles; i++)
          {
            connect(j);
            json_update(POST_MAX, value, tmpTime, false);
          }
          num_post = updates - cycles * POST_MAX;
        }
        connect(j);
        json_update(num_post, value, tmpTime, true);
#if debugEnabled
        if (!ambient__.debug_state()){ Serial.print(F("Posted to Server host : "));Serial.println(j);}
#endif

      }
      else
      {
#if debugEnabled
        if (!ambient__.debug_state())
        {   Serial.println(F("Error updating time Server..!"));   }
#endif
      }
#if debugEnabled
      if (!ambient__.debug_state()) Serial.println(F("Old connection active. Closing..."));
#endif
      if(wificonnected && j==(nb_host-1)) base__.close();
    }
    else //No connect
    {
      if (base__.checkRTC()) base__.RTCtime(time);
      else time = "#";

      if (j==0) addFIFO(value, time);
#if debugEnabled
      if (!ambient__.debug_state()) {
        Serial.println(F("Error in connection! Data saved in memory"));
        Serial.print(F("Pending updates: "));
        Serial.println(updates + 1);
      }
#endif
    }
    if (sleep)
    {
      base__.sleep();
#if debugEnabled
      if (!ambient__.debug_state())
      {
        Serial.println(F("SCK Sleeping"));
      }
#endif
      digitalWrite(AWAKE, LOW);
    }
  }
  else
  {
    if (base__.checkRTC()) base__.RTCtime(time);
    else time = "#";
    if (j==0) addFIFO(value, time);
#if debugEnabled
    if (!ambient__.debug_state()) Serial.println(F("Saved in memory!!"));
#endif
  }
}
  *wait_moment = false;
}

