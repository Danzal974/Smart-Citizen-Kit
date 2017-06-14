/*

  SCKServer.cpp

*/

#include <Wire.h>
#include <EEPROM.h>
#include "SCKServer.h"

SCKServer::SCKServer(SCKBase& base)
{
  _base = base;
}

#define numbers_retry 5

boolean SCKServer::time(char *time_)
{
  boolean ok = false;
  uint8_t count = 0;
  byte retry = 0;
  byte webtime = 0; //0 : smartcitizen ; 1 : communecter
  while ((retry < numbers_retry) && (!ok)) {
    webtime = retry % HOSTS;
    retry++;

    //if (_base.enterCommandMode()) {
    if (_base.ready()) {
      if (_base.open(HOSTADDR[webtime], 80)) {
        //Requests to the server time
        Serial1.print("GET ");
        Serial1.print(TIMEENDPOINT[webtime]);
        Serial1.print(WEB[0]);
        Serial1.print(HOSTADDR[webtime]);
        //Serial1.print(" ");
        Serial1.println(WEB[1]);
        //Serial1.println(F("\r\n"));
#if debugServer
        Serial.print(F("GET "));
        Serial.print(TIMEENDPOINT[webtime]);
        Serial.print(WEB[0]);
        Serial.print(HOSTADDR[webtime]);
        //Serial1.print(" ");
        Serial.println(WEB[1]);
        Serial.flush();
#endif
        if (_base.findInResponse(WEB200OK, 2000)) {
          if (_base.findInResponse("UTC:", 2000)) {
            char newChar;
            byte offset = 0;
            unsigned long time = millis();
            while (offset < TIME_BUFFER_SIZE) {
              if (Serial1.available()) {
                newChar = Serial1.read();
#if debugServer
                Serial.print(newChar);
#endif
                time = millis();
                if (newChar == '#') {
                  ok = true;
                  time_[offset] = '\x00';
                  break;
                }
                else if (newChar != -1) {
                  if (newChar == ',') {
                    if (count < 2) time_[offset] = '-';
                    else if (count > 2) time_[offset] = ':';
                    else time_[offset] = ' ';
                    count++;
                  }
                  else time_[offset] = newChar;
                  offset++;
                }
                else time_[offset] = newChar;
                offset++;
              }
              else if ((millis() - time) > 2000) {
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
        }
        _base.close();
      }
    }
  }
  if (!ok) {
    time_[0] = '#';
    time_[1] = 0x00;
  }
  //_base.exitCommandMode();
  return ok;
}

boolean SCKServer::RTCupdate(char *time_)
{
  byte retry = 0;
  if (_base.checkRTC()) {
    if (time(time_)) {
      while (retry < numbers_retry) {
        retry++;
        if (_base.RTCadjust(time_)) {
          return true;
        }
      }
    }
  }
  return false;
}

void SCKServer::json_update(uint16_t updates, byte host, long *value, char *time, boolean isMultipart)
{
#if debugServer
  Serial.print(F("["));
#endif
  Serial1.print(F("["));
  for (int i = 0; i < updates; i++) {
    readFIFO(host);
    if ((i < (updates - 1)) || (isMultipart)) {
      Serial1.print(F(","));
#if debugServer
      Serial.print(F(","));
#endif
    }
  }

  if (isMultipart) {
    byte i;
    for (i = 0; i < SENSORS; i++) {
      Serial1.print(SERVER[i]);
      Serial1.print(value[i]);
    }
    Serial1.print(SERVER[i]);
    Serial1.print(time);
    Serial1.print(SERVER[i + 1]);
    Serial1.println(F("]"));
    Serial1.println();

#if debugServer
    for (i = 0; i < SENSORS; i++) {
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
  uint16_t updates = (_base.readData(EE_ADDR_NUMBER_WRITE_MEASURE, INTERNAL) - _base.readData(EE_ADDR_NUMBER_READ_MEASURE, INTERNAL)) / ((SENSORS) * 4 + TIME_BUFFER_SIZE);
  if (updates < MAX_MEMORY) {
    int eeaddress = _base.readData(EE_ADDR_NUMBER_WRITE_MEASURE, INTERNAL);
    int i = 0;
    for (i = 0; i < SENSORS; i++) {
      _base.writeData(eeaddress + i * 4, value[i], EXTERNAL);
    }
    _base.writeData(eeaddress + i * 4, 0, time, EXTERNAL);
    eeaddress = eeaddress + (SENSORS) * 4 + TIME_BUFFER_SIZE;
    _base.writeData(EE_ADDR_NUMBER_WRITE_MEASURE, eeaddress, INTERNAL);
  }
  else {
#if debugEnabled
    if (_base.getDebugState()) Serial.println(F("Memory limit exceeded!!"));
#endif
  }
}

void SCKServer::readFIFO(byte host)
{
  int i = 0;
  int eeaddressread = _base.readData(EE_ADDR_NUMBER_READ_MEASURE + (host * 4), INTERNAL);
  for (i = 0; i < SENSORS; i++) {
    Serial1.print(SERVER[i]);
    Serial1.print(_base.readData(eeaddressread + i * 4, EXTERNAL)); //SENSORS
  }
  Serial1.print(SERVER[i]);
  Serial1.print(_base.readData(eeaddressread + i * 4, 0, EXTERNAL)); //TIME
  Serial1.print(SERVER[i + 1]);

#if debugServer
  for (i = 0; i < SENSORS; i++) {
    Serial.print(SERVER[i]);
    Serial.print(_base.readData(eeaddressread + i * 4, EXTERNAL)); //SENSORS
  }
  Serial.print(SERVER[i]);
  Serial.print(_base.readData(eeaddressread + i * 4, 0, EXTERNAL)); //TIME
  Serial.print(SERVER[i + 1]);
#endif
  int eeaddresswrite = _base.readData(EE_ADDR_NUMBER_WRITE_MEASURE, INTERNAL);
#if debugServer
  Serial.print(F("\nWrite Address : "));
  Serial.println(eeaddresswrite);
#endif
  /*
    eeaddressread = eeaddressread + (SENSORS * 4) + TIME_BUFFER_SIZE; // Next Measure to read
    if (host < (HOSTS - 1)) {
      _base.writeData(EE_ADDR_NUMBER_READ_MEASURE + (host * 4), eeaddress, INTERNAL);
    }
    else if ( (_base.readData(EE_ADDR_NUMBER_READ_MEASURE, INTERNAL) + (SENSORS * 4) + TIME_BUFFER_SIZE) == eeaddresswrite) {
      _base.writeData(EE_ADDR_NUMBER_WRITE_MEASURE, 0, INTERNAL);
      _base.writeData(EE_ADDR_NUMBER_READ_MEASURE, 0, INTERNAL);
    }
    else _base.writeData(EE_ADDR_NUMBER_READ_MEASURE + (host * 4), eeaddress, INTERNAL);
  */
  // Set the Next address to read
  eeaddressread = eeaddressread + (SENSORS * 4) + TIME_BUFFER_SIZE;
  _base.writeData(EE_ADDR_NUMBER_READ_MEASURE + (host * 4), eeaddressread, INTERNAL);
#if debugServer
  Serial.print(F("Next Address : "));
  Serial.println(eeaddressread);
#endif
  boolean sameNextMeasure = true;
  for (i = 0; i < (HOSTS - 1) && sameNextMeasure; i++) {
    if (_base.readData(EE_ADDR_NUMBER_READ_MEASURE + (i * 4), INTERNAL) != _base.readData(EE_ADDR_NUMBER_READ_MEASURE + ((i + 1) * 4), INTERNAL)) sameNextMeasure = false;
  }
  if (sameNextMeasure) {
    // Move data in EEPROM
    for (i = eeaddressread; i < eeaddresswrite; i += (SENSORS * 4) + TIME_BUFFER_SIZE) {
      _base.writeData(i - eeaddressread, _base.readData(i, EXTERNAL), EXTERNAL);
    }
    for (i = 0; i < HOSTS ; i++) {
      _base.writeData(EE_ADDR_NUMBER_READ_MEASURE  + (i * 4), 0, INTERNAL);
    }
    _base.writeData(EE_ADDR_NUMBER_WRITE_MEASURE, eeaddresswrite - eeaddressread, INTERNAL);
  }

}



boolean SCKServer::update(long *value, char *time_)
{
  value[8] = _base.scan();  //Wifi Nets
  byte retry = 0;
  if (time(time_)) {
    //Update server time
    if (_base.checkRTC()) {
      while (!_base.RTCadjust(time_) && (retry < numbers_retry)) retry++;
    }
  }
  else if (_base.checkRTC()) {
    _base.RTCtime(time_);
#if debugEnabled
    if (_base.getDebugState()) Serial.println(F("Fail server time!!"));
#endif
  }
  else {
    time_ = "#";
    return false;
  }
  return true;
}

boolean SCKServer::connect(byte webhost)
{
  int retry = 0;

  while (true) {
    if (_base.open(HOSTADDR[webhost], 80)) break;
    else {
      retry++;
      if (retry >= numbers_retry) return false;
    }
  }
  Serial1.print("PUT ");
  Serial1.print(ENDPTHTTP[webhost]); // in Constants
  Serial1.print(WEB[0]);
  Serial1.print(HOSTADDR[webhost]); //
  Serial1.print(WEB[1]);
  if (webhost == 1) Serial1.print(AUTHPH); // required for communecter.org
  Serial1.print(WEB[2]);
  Serial1.println(_base.readData(EE_ADDR_MAC, 0, INTERNAL)); //MAC ADDRESS
  Serial1.print(WEB[3]);
  Serial1.println(_base.readData(EE_ADDR_APIKEY, 0, INTERNAL)); //Apikey
  Serial1.print(WEB[4]);
  Serial1.println(FirmWare); //Firmware version
  Serial1.print(WEB[5]);

#if debugServer
  if (_base.getDebugState()) {
    Serial.print(F("PUT "));
    Serial.print(ENDPTHTTP[webhost]); // in Constants
    Serial.print(WEB[0]);
    Serial.print(HOSTADDR[webhost]); //
    Serial.print(WEB[1]);
    if (webhost == 1) Serial.print(AUTHPH); // required for communecter.org
    Serial.print(WEB[2]);
    Serial.println(_base.readData(EE_ADDR_MAC, 0, INTERNAL)); //MAC ADDRESS
    Serial.print(WEB[3]);
    Serial.println(_base.readData(EE_ADDR_APIKEY, 0, INTERNAL)); //Apikey
    Serial.print(WEB[4]);
    Serial.println(FirmWare); //Firmware version
    Serial.print(WEB[5]);
    Serial.println();
  }
#endif

  //if (_base.findInResponse(WEB200OK, 2000)) return true;
  //return false;
  return true;
}


void SCKServer::send(boolean sleep, boolean *wait_moment, long *value, char *time, boolean instant)
{
  *wait_moment = true;
  if (_base.checkRTC()) _base.RTCtime(time);
  char tmpTime[19];
  strncpy(tmpTime, time, 20);
  uint16_t updates = (_base.readData(EE_ADDR_NUMBER_WRITE_MEASURE, INTERNAL) - _base.readData(EE_ADDR_NUMBER_READ_MEASURE, INTERNAL)) / ((SENSORS) * 4 + TIME_BUFFER_SIZE);
  uint16_t NumUpdates = _base.readData(EE_ADDR_NUMBER_UPDATES, INTERNAL); // Number of readings before batch update
  Serial.flush();
  Serial.print(MSGCONST[11]);
  Serial.println(updates);
  if (updates >= (NumUpdates - 1) || instant) {
    if (sleep) {
#if debugEnabled
      if (_base.getDebugState()) {
        Serial.println(F("SCK Waking up..."));
      }
#endif
      digitalWrite(AWAKE, HIGH);
    }
    if (_base.connect()) {
      //Wifi connect
#if debugEnabled
      if (_base.getDebugState()) Serial.println(MSGCONST[8]);
#endif
      if (update(value, time)) {
        //Update time and nets

        // POST DATA
        boolean connection_failed[HOSTS];
        for (byte j = 0 ; j < HOSTS /*&& !connection_failed*/; j++) {
          // post data to each host
          connection_failed[j] = false;
          updates = (_base.readData(EE_ADDR_NUMBER_WRITE_MEASURE, INTERNAL) - _base.readData(EE_ADDR_NUMBER_READ_MEASURE + (j * 4), INTERNAL)) / ((SENSORS) * 4 + TIME_BUFFER_SIZE);
#if debugEnabled && debugServer
          if (_base.getDebugState()) {
            Serial.print(HOSTADDR[j]);
            Serial.print(F(" : updates = "));
            Serial.println(updates + 1);
          }
#endif
          int num_post = updates;
          int cycles = updates / POST_MAX;
          if (updates > POST_MAX) {
            for (int i = 0; i < cycles; i++) {
              if (connect(j)) json_update(POST_MAX, j, value, tmpTime, false);
              else {
                connection_failed[j] = true;
                break; //skip all transmission
              }
            }
            num_post = updates - cycles * POST_MAX;
          }
          if (connect(j) && !connection_failed[j]) json_update(num_post, j, value, tmpTime, true);
          else connection_failed[j] = true;          
          if (!connection_failed[j]) {
#if debugEnabled
            if (_base.getDebugState()) {
              Serial.print(HOSTADDR[j]);
              Serial.println(MSGCONST[10]);
            }
#endif
          }
          else {
#if debugEnabled
            if (_base.getDebugState()) {
              Serial.print(HOSTADDR[j]);
              Serial.println(F(" NOT")); Serial.println(MSGCONST[10]);
            }
#endif
          }
        }
        boolean data_stored = false;
        for (byte j = 0 ; j < HOSTS; j++) {
          if (connection_failed[j]) {
            if (_base.checkRTC()) _base.RTCtime(time);
            else time = "#";
            if (!data_stored) {
              addFIFO(value, time);
              data_stored = true;
#if debugEnabled
              if (_base.getDebugState()) {
                Serial.println(MSGCONST[9]);
              }
#endif
            }
          }
          else if (data_stored) {
            _base.writeData( EE_ADDR_NUMBER_READ_MEASURE + (j * 4), _base.readData(EE_ADDR_NUMBER_READ_MEASURE + (j * 4), INTERNAL) + SENSORS * 4 + TIME_BUFFER_SIZE, INTERNAL);
          }
        }
      }
#if debugEnabled
      else if (_base.getDebugState()) {
        Serial.println(F("Err up time Server!"));
      }
#endif

#if debugEnabled
      if (_base.getDebugState()) Serial.println(F("Closing old CO.."));
#endif
      _base.close();
    }
    else {
      //No connect
      if (_base.checkRTC()) _base.RTCtime(time);
      else time = "#";
      addFIFO(value, time);
#if debugEnabled
      if (_base.getDebugState()) {
        Serial.print(F("Err in CO!"));
        Serial.println(MSGCONST[9]);
        Serial.print(F("Pending updates:"));
        Serial.println(updates + 1);
      }
#endif
    }
    if (sleep) {
      _base.sleep();
#if debugEnabled
      if (_base.getDebugState()) Serial.println(F("SCK Sleeping"));
#endif
      digitalWrite(AWAKE, LOW);
    }
  }
  else {
    if (_base.checkRTC()) _base.RTCtime(time);
    else time = "#";
    addFIFO(value, time);
#if debugEnabled
    if (_base.getDebugState()) Serial.println(MSGCONST[9]);
#endif
  }
  *wait_moment = false;
}

