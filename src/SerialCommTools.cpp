/**
 * SMuFF Firmware
 * Copyright (C) 2019-2024 Technik Gegg
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 */

/*
    Supporting functions for the communication over serial ports.
*/

#include "SMuFF.h"

String                    serialBuffer0, serialBuffer1, serialBuffer2, serialBuffer3;
volatile uint16_t         bracketCnt = 0;
volatile uint16_t         jsonPtr = 0;
String                    jsonData;
uint32_t                  uploadStart = 0;
uint32_t                  lastEvent = 0;
bool                      isQuote = false;
bool                      isFuncKey = false;
bool                      isCtlKey = false;
bool                      ignoreQuotes = false;
volatile bool             isUpload = false;
bool                      isESPdebug = false;

void checkSerialPending() {
  if (Serial.available()) {
    serialEvent();
    lastEvent = millis();
  }
  if (CAN_USE_SERIAL1) {
    if (Serial1.available()) {
      serialEvent1();
      lastEvent = millis();
    }
  }
  if (CAN_USE_SERIAL2) {
    if (Serial2.available()) {
      serialEvent2();
      lastEvent = millis();
    }
  }
  if (CAN_USE_SERIAL3) {
    if (Serial3.available()) {
      serialEvent3();
      lastEvent = millis();
    }
  }
}

void resetSerialBuffer(int8_t serial) {
  switch (serial) {
    case 0:
      serialBuffer0 = "";
      break;
    case 1:
      serialBuffer1 = "";
      break;
    case 2:
      serialBuffer2 = "";
      break;
    case 3:
      serialBuffer3 = "";
      break;
  }
}

void filterSerialInput(String &buffer, char in) {
  // function key sequence starts with 'ESC['
  if (!isFuncKey && in == 0x1b) {
    isFuncKey = true;
    return;
  }
  if (isFuncKey) {
    //__debugS(D, PSTR("%02x"), in);
    if (in == 'P') { // second escape char 'P' - swallow that, set ignore quotes
      ignoreQuotes = true;
      isFuncKey = false;
      return;
    }
    if (in == 'U') { // second escape char 'U' - swallow that, set isUpload flag
      drawUpload(uploadLen);
      isUpload = true;
      parserBusy = true;
      isFuncKey = false;
      uploadStart = 0;
      return;
    }
    if (in == 'D') { // second escape char 'D' - swallow that, set isESPdebug flag
      isESPdebug = true;
      isFuncKey = false;
      return;
    }

    if (in == '[' || in == 'O') // second escape char '[' or 'O' - swallow that
      return;
    isFuncKey = false;
    switch (in) {
      case 0x42:
        remoteKey = REMOTE_UP;
        return; // CursorUp   = turn right
      case 0x41:
        remoteKey = REMOTE_DOWN;
        return; // CursorDown  = turn left
      case 0x43:
        remoteKey = REMOTE_SELECT;
        return;  // CursorRight = wheel click
      case 0x1b: // ESC Key
      case 0x44:
        remoteKey = REMOTE_ESCAPE;
        return; // CursorLeft = main click
      case 0x31:
        remoteKey = REMOTE_HOME;
        return; // Home Key
      case 0x34:
        remoteKey = REMOTE_END;
        return; // End Key  (not used yet)
      case 0x35:
        remoteKey = REMOTE_PGUP;
        return; // PageUp Key (not used yet)
      case 0x36:
        remoteKey = REMOTE_PGDN;
        return; // PageDown Key (not used yet)
      case 0x50:
        remoteKey = REMOTE_PF1;
        return; // F1 Key = fncKey1()
      case 0x51:
        remoteKey = REMOTE_PF2;
        return; // F2 Key = fncKey2()
      case 0x52:
        remoteKey = REMOTE_PF3;
        return; // F3 Key = fncKey3()
      case 0x53:
        remoteKey = REMOTE_PF4;
        return; // F4 Key = fncKey4()
      default:
        return; // ignore any other code not in the list
    }
  }
  isFuncKey = false;
  // special function for Duet3D: if "\n" is transmitted (two characters)
  // then threat that as a line-feed eventually. Otherwise if it's a "\\"
  // store that as a single "\" in the buffer or if it's a "\s" ignore that
  // control string (used in earlier versions of the Duet3D in conjunction with SMuFF-Ifc).
  if (in == '\\') {
    if (isCtlKey) {
      isCtlKey = false;
      if(in != 's')       // ignore a '\s'
        buffer += in;
    }
    else {
      isCtlKey = true;
    }
    return;
  }
  if (in >= 'a' && in <= 'z') {
    if (!isQuote && !ignoreQuotes)
      in = in - 0x20;
  }
  switch (in) {
    case '\b': {
        if(buffer.substring(buffer.length() - 1)=="\"")
          isQuote = !isQuote;
        buffer = buffer.substring(0, buffer.length() - 1);
      }
      break;
    case '\r':
      break;
    case '"':
      isQuote = !isQuote;
      buffer += in;
      break;
    case ' ':
      if (isQuote)
        buffer += in;
      break;
    default:
      if (buffer.length() < 4096) {
        if (in >= 0x21 && in <= 0x7e) // read over non-ascii characters, just in case
          buffer += in;
      }
      else {
        __debugS(W, PSTR("Buffer exceeded 4096 bytes!"));
      }
      break;
  }
}

void sendToDuet3D(char in) {
  switch (smuffConfig.duet3Dport) {
    case 0:
      return;
    case 1:
      if (CAN_USE_SERIAL1)
        Serial1.write(in);
      break;
    case 2:
      if (CAN_USE_SERIAL2)
        Serial2.write(in);
      break;
    case 3:
      if (CAN_USE_SERIAL3)
        Serial3.write(in);
      break;
  }
}

void sendToPanelDue(char in) {
  // only if PanelDue is configured...
  switch (smuffConfig.hasPanelDue) {
    case 0:
      return;
    case 1:
      if (CAN_USE_SERIAL1)
        Serial1.write(in);
      break;
    case 2:
      if (CAN_USE_SERIAL2)
        Serial2.write(in);
      break;
    case 3:
      if (CAN_USE_SERIAL3)
        Serial3.write(in);
      break;
  }
}

bool isJsonData(char in, uint8_t port) {
  // check for JSON formatted data
  if (in == '{') {
    bracketCnt++;
  }
  else if (in == '}') {
    bracketCnt--;
  }
  if (bracketCnt > 0) {
    jsonPtr++;
    //__debugS(D, PSTR("JSON nesting level: %d"), bracketCnt);
  }

  if (jsonPtr > 0) {
    if(port == smuffConfig.displaySerial) {
      // JSON data is coming in from Serial display port
      jsonData += in;
    }
    else if(port == smuffConfig.duet3Dport) {
      // JSON data is coming in from Duet3D port
      // send to PanelDue
      sendToPanelDue(in);
    }
    else if(port == smuffConfig.hasPanelDue) { 
      // JSON data is coming in from PanelDue
      sendToDuet3D(in);
    }
    if (bracketCnt > 0)
      return true;
  }
  if (bracketCnt == 0 && jsonPtr > 0) {
    jsonPtr = 0;
    return true;
  }
  return false;
}

void resetUpload() {
  upload.close();
  isUpload = false;
  gotFirmware = false;
  parserBusy = false;
  uploadStart = 0;
}

void handleUpload(const char* buffer, size_t len, Stream* serial) {
  if(uploadStart > 0 && millis()-uploadStart > 10000) {
    resetUpload();
    __debugS(W, PSTR("Upload aborted because of timeout"), firmware);
    return;
  }
  if (isPwrSave)
    setPwrSave(0);
  drawUpload(uploadLen);
  if(!isUpload)
    return;
  //sendXoff(serial);
  upload.write(buffer, len);
  //upload.flush();
  //sendXon(serial);
  uploadLen -= len;
  uploadStart = millis();

  if(uploadLen <= 0) {
    resetUpload();
    __debugS(I, PSTR("Upload of '%s' finished"), firmware);
  }
}

bool filterESPboot(int port, String buffer) {
  /*
    Filter out any of the following lines, which are sent by the ESP8266/ESP32 at bootup.

    ets Jan  8 2014,rst cause 1, boot mode:(3,7)
    load 0x40100000, len 24236, room 16
    tail 12
    chksum 0xb7
    ho 0 tail 12 room 4
    load 0x3ffe8000, len 3008, room 12
    tail 4
    chksum 0x2c
    load 0x3ffe8bc0, len 4816, room 4
    tail 12
    chksum 0x46
    csum 0x46
  */
  if (buffer.startsWith("ETS") || buffer.substring(1).startsWith("ETS") ||
      buffer.startsWith("LOAD") ||
      buffer.startsWith("TAIL") ||
      buffer.startsWith("CHKSUM") ||
      buffer.startsWith("HO") ||
      buffer.startsWith("CSUM") || 
      buffer.startsWith("USEREXCEPTION") ||
      buffer.startsWith("---------------") ||
      buffer.startsWith("~LD") ||
      buffer.startsWith("LASTFAILED"))
  {
    buffer.replace("\n","\\n");
    buffer.replace("\r","\\r");
    __debugS(DEV4, PSTR("Serial-%d has received ESP boot message: \"%s\""), port, buffer.c_str());
    return true;
  }
  return false;
}

void handleSerial(const char* in, size_t len, String& buffer, uint8_t port) {
  for(size_t i=0; i< len; i++) {
    // handle Ctrl-C sequence, which will interrupt a running test
    if (in[i] == 0x03) {
      isTestrun = false;
      resetSerialBuffer(port);
      __debugS(DEV, PSTR("Ctrl-C received from port %d"), port);
      return;
    }
    // do not proceed any further if a test is running
    if(isTestrun)
      return;
    if(port == smuffConfig.displaySerial) {
      if(isJsonData(in[i], port)) {
        continue;
      }
    }
    // parse for JSON data coming from Duet3D
    if(port == smuffConfig.duet3Dport) {
      if(isJsonData(in[i], port)) {
        continue;
      }
    }
    if (in[i] == '\n' || (isCtlKey && in[i] == 'n')) {
      if(port == 2) {
        if(filterESPboot(2, String(buffer))) {
          resetSerialBuffer(port);
          return;
        }
      }
      if(jsonData != "") {
        uint32_t id = parseJson(jsonData);
        if(id > 0) {
          __debugS(DEV3, PSTR("Got JSON data on port %d [ DlgId: %d, Button: %d ]"), port, id, dlgButton);
          if(id == waitForDlgId) {
            gotDlgId = true;
          }
        }
        else {
          __debugS(DEV3, PSTR("Got invalid JSON data on port %d [ %s ]"), port, jsonData.c_str());
        }
        jsonData = "";
      }
      else {
        if(!isESPdebug) {
          isIdle = false;
          setFastLEDStatus(FASTLED_STAT_NONE);
          __debugS(DEV4, PSTR("Serial-%d has sent \"%s\""), port, buffer.c_str());
          parseGcode(buffer, port);
        }
      }
      isQuote = false;
      actionOk = false;
      isCtlKey = false;
      ignoreQuotes = false;
    }
    else if(isCtlKey && in[i] == '"') {
      // don't handle quotes if they are escaped
      isCtlKey = false;
      buffer += '\\';
      buffer += in[i];
      return;
    }
    else {
      if(isESPdebug) {
        if (in[i] == 0x1a) {
          __debugS(DEV4, PSTR(">> Debug:\n--- ESP --------\n%s\n----------------\n"), buffer.c_str());
          isESPdebug = false;
          resetSerialBuffer(2);
        }
        else {
          buffer += in[i];
        }
      }
      else 
        filterSerialInput(buffer, in[i]);
    }
  }
}

size_t readSerialToBuffer(Stream* serial, char* buffer, size_t maxLen) {
  size_t got = 0;
  int avail = serial->available();
  if(avail != -1) {
    size_t len = avail;
    if(len > maxLen)
      len = maxLen;
    got = serial->readBytes(buffer, len);
  }
  return got;
}

void serialEvent() {  // USB-Serial port
  char tmp[256];
  memset(tmp, 0, ArraySize(tmp));
  size_t got = readSerialToBuffer(&Serial, tmp, ArraySize(tmp)-1);
  if(got > 0) {
    if(isUpload)
      handleUpload(tmp, got, &Serial);
    else {
      if(smuffConfig.useDuet) {
        if(smuffConfig.traceUSBTraffic)
          __debugS(DEV4, PSTR("Recv(0): %s"), tmp);
      }
      handleSerial(tmp, got, serialBuffer0, 0);
    }
  }
}

void serialEvent1() {
  char tmp[256];
  memset(tmp, 0, ArraySize(tmp));
  size_t got = readSerialToBuffer(&Serial1, tmp, ArraySize(tmp)-1);
  if(got > 0) {
    if(isUpload)
      handleUpload(tmp, got, &Serial1);
    else
      handleSerial(tmp, got, serialBuffer1, 1);
  }
}

void serialEvent2() {
  char tmp[256];
  memset(tmp, 0, ArraySize(tmp));
  size_t got = readSerialToBuffer(&Serial2, tmp, ArraySize(tmp)-1);
  if(got > 0) {
    if(isUpload)
      handleUpload(tmp, got, &Serial2);
    else {
      if(smuffConfig.useDuet) {
        if(smuffConfig.traceUSBTraffic)
          __debugS(I, PSTR("Recv(2): %s"), tmp);
      }
      handleSerial(tmp, got, serialBuffer2, 2);
    }
  }
}

void serialEvent3() {
  char tmp[256];
  memset(tmp, 0, ArraySize(tmp));
  size_t got = readSerialToBuffer(&Serial3, tmp, ArraySize(tmp)-1);
  if(got > 0) {
    if(isUpload)
      handleUpload(tmp, got, &Serial3);
    else
      handleSerial(tmp, got, serialBuffer3, 3);
  }
}
