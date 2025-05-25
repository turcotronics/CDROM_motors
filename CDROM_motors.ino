//CDROM_motors
// Turcotronics, License GPL Version 3
// https://www.turcotronics.it
// You can connect directly to port 80 IP 192.168.4.1 or use websockets to port 81
// See example CDROM_motors.html
// The used board is the ESP32 DevkitC V4 but it can be easily adapted to any ESP32 board
// Check out it on https://www.ebay.it/usr/turcotronics or use your own one :-)
// Absolute position in based on time, so it isn't precise, return backward to zeros.
// Default WiFi cSSID: CDROM_motors
// Default WiFi cPASSWORD: 1234567890
// 2 motors, IR Tx/Rx sensor, WS2812 RGB LED
// DO NOT use the ESP32 USB connector to power the motors, it's just for programming
// Choice in the below #define the Kit number or ports and constants
// Use:
//   WebSockets by Markus Sattler
//   Freenove WS2812 Lib for ESP32 by Freenove

// Load Wi-Fi library
#include <SPI.h>
#include <EEPROM.h>
#include <Arduino.h>
#include <WiFi.h>
#include <WebSocketsServer.h>
#include "esp_system.h"
#include <esp_pm.h>
#include <esp_wifi.h>
#include <esp_wifi_types.h>
#include <esp_task_wdt.h>
#include "Freenove_WS2812_Lib_for_ESP32.h"

#define FwVersion "Ver. 1.4"

//Begin User defines -----------------------------------------------------------
#define KIT1 //TODO change this if necessary or define your constants

#ifdef KIT1
#define FORWARDSTOPTIME 8200//Kit 1 // Time to go from position 0 to 100 and I/O Pins//Default stored on EEPROM
#endif
#ifdef KIT2
#define FORWARDSTOPTIME 4200//Kit 2 // Time to go from position 0 to 100 and I/O Pins//Default stored on EEPROM
#endif
#ifdef KIT3
#define FORWARDSTOPTIME 5000//Kit 3 // Time to go from position 0 to 100 and I/O Pins//Default stored on EEPROM
#endif

//PINs definition
#define ENDSTOPPIN   34
#define MOTORLINFWRD 16
#define MOTORLINBACK 17
#define MOTORROTFWRD 32
#define MOTORROTBACK 33
#define OPTSENSEPIN  35
#define COLORLEDPIN  27
#define LED_TYPE LED_STRIP_WS2812
#define LED_TYPE_IS_RGBW 0   // if the LED is an RGBW type, change the 0 to 1

// Replace with your network credentials, default IP 192.168.4.1
// Remove the PASSWORD parameter (set to ""), if you want the AP (Access Point) to be open
#define SSID  "CDROM_motors_1"//Default stored on EEPROM
#define PASSWORD  "1234567890"//Default stored on EEPROM
//End User defines -----------------------------------------------------------

#define EEPROM_FORWARDSTOPTIME 0
#define EEPROM_SSID 2
#define EEPROM_PASSWORD 66
#define EEPROM_LEN 130

//Motors status
#define MOTORSTOP     0
#define MOTORFORWARD  1
#define MOTORBACKWARD 2

//Stop status
#define NOSTOP  0
#define ENDSTOP 1
#define FORSTOP 2

Freenove_ESP32_WS2812 strip = Freenove_ESP32_WS2812(1, COLORLEDPIN, 0, TYPE_GRB);

// Set web server port number to 80
WiFiServer server(80);
WebSocketsServer webSocket = WebSocketsServer(81);

// Variable to store the HTTP request
String header;
String dest;
String dest2;

int MotorRot_Run = MOTORSTOP;
int MotorLinPerc = -1;
int Percentance = 0;
int ix=0;
volatile int MotorLin_Run = MOTORSTOP;
volatile int MotorLinTime = 0;
volatile int MotorLinTimeGo = -1;
volatile int EndStopVal = 0;
volatile int OptiSenseVal = 0;
volatile int OptiSenseValOld = 0;
char Status[4] = "000";  //Status to send to the web socket: Stop(Stop status), MotorLin(Motors status), MotorRot(Motors status)
char Sensors[3] = "S0";  //Sensors
char charBuffer[256];
static uint16_t uiFORWARDSTOPTIME=FORWARDSTOPTIME;//on EEPROM
static char cSSID[64] = SSID;//on EEPROM
static char cPASSWORD[64] = PASSWORD;//on EEPROM
bool bConnected = false;
bool bUpdateStatus = false;
String str;
char Temp[3] = "00";  //Temporary
bool bMotorLinForward=false;
bool bMotorLinBackward=false;
bool bMotorLinStop=false;
bool bMotorRotForward=false;
bool bMotorRotBackward=false;
bool bMotorRotStop=false;
esp_reset_reason_t BootReason = ESP_RST_UNKNOWN;
unsigned long currentMillis = 0;
unsigned long previousMillis = 0;

uint8_t LED_BRIGHT_Val=0;
uint8_t LED_RED_Val=0;
uint8_t LED_GREEN_Val=0;
uint8_t LED_BLUE_Val=0;

hw_timer_t* timer = NULL;
portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;

const esp_task_wdt_config_t WDT_config = {
    .timeout_ms = 4000,
    .idle_core_mask = (1 << portNUM_PROCESSORS) - 1,  // Bitmask of all cores, https://github.com/espressif/esp-idf/blob/v5.2.2/examples/system/task_watchdog/main/task_watchdog_example_main.c
    .trigger_panic = true                             // Enable panic to restart ESP32
  };

//--------------------------------------------
void ARDUINO_ISR_ATTR onTimer() {
  // portENTER_CRITICAL_ISR(&timerMux);
  //checks end stop
  EndStopVal = !digitalRead(ENDSTOPPIN);
  // portEXIT_CRITICAL_ISR(&timerMux);

  if (EndStopVal == 1)  //Stop the motor if end reached
  {
    MotorLinTime = 0;
    if (MotorLin_Run == MOTORBACKWARD)  //backward
    {
      MotorLin_Run = MOTORSTOP;
      bUpdateStatus = true;
      digitalWrite(MOTORLINFWRD, HIGH);  //linear motor
      digitalWrite(MOTORLINBACK, HIGH);
    }
  } else {
    if (MotorLin_Run == MOTORFORWARD)  //forward
    {
      //Cont time to stop before the forward end
      MotorLinTime=MotorLinTime+1;
      if (MotorLinTime >= uiFORWARDSTOPTIME) {
        MotorLinTime = uiFORWARDSTOPTIME;
        MotorLin_Run = MOTORSTOP;
        bUpdateStatus = true;
        digitalWrite(MOTORLINFWRD, HIGH);  //linear motor
        digitalWrite(MOTORLINBACK, HIGH);
      }
    } else if (MotorLin_Run == MOTORBACKWARD)  //backward
    {
      MotorLinTime=MotorLinTime-1;
      if (MotorLinTime < 0)
        MotorLinTime = 0;
    }
  }
  if (MotorLinTimeGo != -1) {
    if (MotorLin_Run == MOTORFORWARD) {
      if (MotorLinTime >= MotorLinTimeGo) {
        MotorLinTimeGo = -1;
        MotorLin_Run = MOTORSTOP;
        bUpdateStatus = true;
        digitalWrite(MOTORLINFWRD, HIGH);  //linear motor
        digitalWrite(MOTORLINBACK, HIGH);
      }
    } else if (MotorLin_Run == MOTORBACKWARD) {
      if (MotorLinTime <= MotorLinTimeGo) {
        MotorLinTimeGo = -1;
        MotorLin_Run = MOTORSTOP;
        bUpdateStatus = true;
        digitalWrite(MOTORLINFWRD, HIGH);  //linear motor
        digitalWrite(MOTORLINBACK, HIGH);
      }
    }
  }
}

//--------------------------------------------
void MotorLinStop() {
  Serial.println("MotorLin stop");
  MotorLin_Run = MOTORSTOP;  //stop
  bUpdateStatus = true;
  digitalWrite(MOTORLINFWRD, HIGH);  //linear motor
  digitalWrite(MOTORLINBACK, HIGH);
}

//--------------------------------------------
void MotorLinForward() {
  if (MotorLin_Run == MOTORBACKWARD)  //avoid extra current for fast inversion
  {
    MotorLinStop();
    delay(1000);
  }
  if (MotorLinTime < uiFORWARDSTOPTIME)  //run only if not reached the forward end
  {
    Serial.println("MotorLin forward");
    MotorLin_Run = MOTORFORWARD;  //forward
    bUpdateStatus = true;
    digitalWrite(MOTORLINFWRD, HIGH);  //linear motor
    digitalWrite(MOTORLINBACK, LOW);
  }
}

//--------------------------------------------
void MotorLinBackward() {
  if (EndStopVal == 0)  //Run only if the end stop is not reached
  {
    if (MotorLin_Run == MOTORFORWARD)  //avoid extra current for fast inversion
    {
      MotorLinStop();
      delay(1000);
    }
    Serial.println("MotorLin backward");
    MotorLin_Run = MOTORBACKWARD;  //backward
    bUpdateStatus = true;
    digitalWrite(MOTORLINFWRD, LOW);  //linear motor
    digitalWrite(MOTORLINBACK, HIGH);
  }
}

//--------------------------------------------
void MotorRotStop() {
  Serial.println("MotorRot stop");
  MotorRot_Run = MOTORSTOP;  //stop
  bUpdateStatus = true;
  digitalWrite(MOTORROTFWRD, HIGH);  //rotative motor
  digitalWrite(MOTORROTBACK, HIGH);
}

//--------------------------------------------
void MotorRotForward() {
  if (MotorRot_Run == MOTORBACKWARD)  //avoid extra current for fast inversion
  {
    MotorRotStop();
    delay(1000);
  }
  Serial.println("MotorRot forward");
  MotorRot_Run = MOTORFORWARD;  //forward
  bUpdateStatus = true;
  digitalWrite(MOTORROTFWRD, HIGH);  //rotative motor
  digitalWrite(MOTORROTBACK, LOW);
}

//--------------------------------------------
void MotorRotBackward() {
  if (MotorRot_Run == MOTORFORWARD)  //avoid extra current for fast inversion
  {
    MotorRotStop();
    delay(1000);
  }
  Serial.println("MotorRot backward");
  MotorRot_Run = MOTORBACKWARD;  //backward
  bUpdateStatus = true;
  digitalWrite(MOTORROTFWRD, LOW);  //rotative motor
  digitalWrite(MOTORROTBACK, HIGH);
}

//--------------------------------------------
void SendStatus() {
  if (EndStopVal == 1)
    Status[0] = ENDSTOP + 48;
  else if (MotorLinTime >= uiFORWARDSTOPTIME)
    Status[0] = FORSTOP + 48;
  else
    Status[0] = NOSTOP + 48;
  Status[1] = MotorLin_Run + 48;
  Status[2] = MotorRot_Run + 48;
  Status[3] = 0;
  // send message to client
  Serial.println("Send Status to webSockets");
  webSocket.broadcastTXT(Status);
  delay(1);
  str = "P" + String(round(MotorLinTime * 100 / uiFORWARDSTOPTIME), 0);
  webSocket.broadcastTXT(str);
  delay(1);
  Sensors[0] = 'S';
  Sensors[1] = OptiSenseVal + 48;
  Sensors[2] = 0;
  webSocket.broadcastTXT(Sensors);
  sprintf(charBuffer, "W%0x", LED_BRIGHT_Val*0x1000000+LED_RED_Val*0x10000+LED_GREEN_Val*0x100+LED_BLUE_Val);
  webSocket.broadcastTXT(charBuffer);
  delay(1);
}

//--------------------------------------------
void webSocketEvent(uint8_t num, WStype_t type, uint8_t* payload, size_t length) {
  IPAddress ip;
  ip = webSocket.remoteIP(num);

  switch (type) {
    case WStype_DISCONNECTED:
      bConnected = true;
      sprintf(charBuffer, "WebSocket[%u] disconnected\n", num);
      Serial.println(charBuffer);
      break;
    case WStype_CONNECTED:
      bConnected = true;
      sprintf(charBuffer, "WebSocket[%u] connected from %d.%d.%d.%d\n", num, ip[0], ip[1], ip[2], ip[3]);
      Serial.println(charBuffer);
      SendStatus();
      break;
    case WStype_TEXT:
      sprintf(charBuffer, " WebSocket[%u]:%s", num, payload);
      Serial.println(charBuffer);
      if (payload[0] == 'L')  //Motor linear
      {
        if (payload[1] == 'F')
          bMotorLinForward=true;
        else if (payload[1] == 'B')
          bMotorLinBackward=true;
        else if (payload[1] == 'S')
          bMotorLinStop=true;
      }
      else if (payload[0] == 'R')  //Motor rotative
      {
        if (payload[1] == 'F')
          bMotorRotForward=true;
        else if (payload[1] == 'B')
          bMotorRotBackward=true;
        else if (payload[1] == 'S')
          bMotorRotStop=true;
      }
      else if (payload[0] == 'P')  //Motor linear %
      {
        MotorLinPerc = atoi((char*)(payload + 1));
      }
      else if (payload[0] == 'W')  //Led %
      {
        Temp[2] = 0;
        Temp[0] = payload[1];
        Temp[1] = payload[2];
        LED_BRIGHT_Val = (uint8_t)strtol(Temp, NULL, 16);
        Temp[0] = payload[3];
        Temp[1] = payload[4];
        LED_RED_Val = (uint8_t)strtol(Temp, NULL, 16);
        Temp[0] = payload[5];
        Temp[1] = payload[6];
        LED_GREEN_Val = (uint8_t)strtol(Temp, NULL, 16);
        Temp[0] = payload[7];
        Temp[1] = payload[8];
        LED_BLUE_Val = (uint8_t)strtol(Temp, NULL, 16);
        LightLed();
      }
      break;
    case WStype_BIN:
      //Serial.println("[%u] get binary length: %u\n", num, length);

      // send message to client
      // webSocket.sendBIN(num, payload, length);
      break;
    case WStype_ERROR:
    case WStype_FRAGMENT_TEXT_START:
    case WStype_FRAGMENT_BIN_START:
    case WStype_FRAGMENT:
    case WStype_FRAGMENT_FIN:
      break;
  }
}

//--------------------------------------------
void LightLed()
 {
    Serial.print("Led B");
    Serial.print(LED_BRIGHT_Val);
    Serial.print(" R");
    Serial.print(LED_RED_Val);
    Serial.print(" G");
    Serial.print(LED_GREEN_Val);
    Serial.print(" B");
    Serial.println(LED_BLUE_Val);
    strip.setBrightness(LED_BRIGHT_Val);
    strip.setLedColorData(0, LED_RED_Val, LED_GREEN_Val, LED_BLUE_Val);
    strip.show();
 }

//--------------------------------------------
void setup() {
  // Initialize pins
  pinMode(MOTORLINFWRD, OUTPUT);
  pinMode(MOTORLINBACK, OUTPUT);
  pinMode(MOTORROTFWRD, OUTPUT);
  pinMode(MOTORROTBACK, OUTPUT);
  pinMode(ENDSTOPPIN, INPUT);
  pinMode(OPTSENSEPIN, INPUT);
  pinMode(COLORLEDPIN, OUTPUT);

  digitalWrite(MOTORLINFWRD, HIGH);  //linear motor
  digitalWrite(MOTORLINBACK, HIGH);
  digitalWrite(MOTORROTFWRD, HIGH);  //rotative motor
  digitalWrite(MOTORROTBACK, HIGH);
  EndStopVal = !digitalRead(ENDSTOPPIN);
  OptiSenseVal = !digitalRead(OPTSENSEPIN);
  OptiSenseValOld=OptiSenseVal;
  digitalWrite(COLORLEDPIN, LOW);

  strip.begin();
	strip.setBrightness(0);

  Serial.begin(115200);

  BootReason = esp_reset_reason();
  Serial.print("Reset/Boot Reason was: "); Serial.println( BootReason );
  switch (BootReason) {
        case ESP_RST_UNKNOWN:
          Serial.println("Reset reason can not be determined");
        break;

        case ESP_RST_POWERON:
          Serial.println("Reset due to power-on event");
        break;

        case ESP_RST_EXT:
          Serial.println("Reset by external pin (not applicable for ESP32)");
        break;

        case ESP_RST_SW:
          Serial.println("Software reset via esp_restart");
        break;

        case ESP_RST_PANIC:
          Serial.println("Software reset due to exception/panic");
        break;

        case ESP_RST_INT_WDT:
          Serial.println("Reset (software or hardware) due to interrupt watchdog");
        break;

        case ESP_RST_TASK_WDT:
          Serial.println("Reset due to task watchdog");
        break;

        case ESP_RST_WDT:
          Serial.println("Reset due to other watchdogs");
        break;                                

        case ESP_RST_DEEPSLEEP:
          Serial.println("Reset after exiting deep sleep mode");
        break;

        case ESP_RST_BROWNOUT:
          Serial.println("Brownout reset (software or hardware)");
        break;
        
        case ESP_RST_SDIO:
          Serial.println("Reset over SDIO");
        break;
        
        default:
          Serial.println("Reset reason unknown");
        break;
    }

  // Data from EEPROM
  EEPROM.begin(EEPROM_LEN);
  EEPROM.get(EEPROM_FORWARDSTOPTIME, uiFORWARDSTOPTIME);
  if((uiFORWARDSTOPTIME==0)||(uiFORWARDSTOPTIME==0xFFFF))
    uiFORWARDSTOPTIME=FORWARDSTOPTIME;
  EEPROM.get(EEPROM_SSID, cSSID);
  if((cSSID[0]=0)||(cSSID[0]=0xFF))
  {
    strlcpy(cSSID,SSID,sizeof(cSSID));//Default if not initialized
    for (ix = 0; ix < sizeof(cSSID); ix++)
    {
      if ((cSSID[ix] < 32) || (cSSID[ix] > 126))
        cSSID[ix] = 0;
    }
    EEPROM.put(EEPROM_SSID, cSSID);
    EEPROM.commit();
  }
  EEPROM.get(EEPROM_PASSWORD, cPASSWORD);
  if((cPASSWORD[0]=0xFF)||((sizeof(PASSWORD)==0)&&(cPASSWORD[0]!=0)))
  {
    strlcpy(cPASSWORD,PASSWORD,sizeof(cPASSWORD));//Default if not initialized
    for (ix = 0; ix < sizeof(cPASSWORD); ix++) {
      if ((cPASSWORD[ix] < 32) || (cPASSWORD[ix] > 126))
        cPASSWORD[ix] = 0;
    }
    EEPROM.put(EEPROM_PASSWORD, cPASSWORD);
    EEPROM.commit();
  }

  // Connect to Wi-Fi network with cSSID and cPASSWORD
  Serial.print("Setting AP (Access Point)…");
  WiFi.softAP(cSSID, cPASSWORD);

  IPAddress IP = WiFi.softAPIP();
  Serial.print("AP IP address: ");
  Serial.println(IP);
  server.begin();
  delay(100);
  webSocket.begin();
  webSocket.onEvent(webSocketEvent);
  delay(100);

  //Initialize timer
  noInterrupts();  // disable all interrupts
  // Motor timer
  timer = timerBegin(1000000);
  timerAttachInterrupt(timer, &onTimer);
  timerAlarm(timer, 100, true, 0);
  interrupts();  // enable all interrupts
  delay(1);

  //Go to the initial position
  MotorLinBackward();
  esp_task_wdt_init(&WDT_config); //enable panic so ESP32 restarts
  esp_task_wdt_add(NULL); //add current thread to WDT watch
}

//--------------------------------------------
void loop() {
  esp_task_wdt_reset();
  delay(1);//mandatory for wdt
  if (bUpdateStatus && bConnected) {
    bUpdateStatus = false;
    SendStatus();
  }

  if(bMotorLinForward)
  {
    bMotorLinForward=false;
    MotorLinForward();
  }
  if(bMotorLinBackward)
  {
    bMotorLinBackward=false;
    MotorLinBackward();
  }
  if(bMotorLinStop)
  {
    bMotorLinStop=false;
    MotorLinStop();
  }
  if(bMotorRotForward)
  {
    bMotorRotForward=false;
    MotorRotForward();
  }
  if(bMotorRotBackward)
  {
    bMotorRotBackward=false;
    MotorRotBackward();
  }
  if(bMotorRotStop)
  {
    bMotorRotStop=false;
    MotorRotStop();
  }

  OptiSenseVal = !digitalRead(OPTSENSEPIN);
 if(OptiSenseValOld!=OptiSenseVal)
  bUpdateStatus = true;
  OptiSenseValOld=OptiSenseVal;

  if (MotorLinPerc != -1)  //MotorLin must go to MotorLinPerc
  {
    if (MotorLinPerc < 0)
      MotorLinPerc = 0;
    else if (MotorLinPerc > 100)
      MotorLinPerc = 100;
    Percentance = round(MotorLinTime * 100 / uiFORWARDSTOPTIME);
    if (MotorLinPerc > Percentance)
      MotorLinForward();
    else if (MotorLinPerc < Percentance)
      MotorLinBackward();
    MotorLinTimeGo = MotorLinPerc * uiFORWARDSTOPTIME / 100;
    MotorLinPerc = -1;
  }
  webSocket.loop();
  delay(1);
  WiFiClient client = server.accept();  // Listen for incoming clients

  if (client) {                     // If a new client connects,
    Serial.println("New Client.");  // print a message out in the serial port
    String currentLine = "";        // make a String to hold incoming data from the client
    currentMillis = millis();
    previousMillis=currentMillis;
    while (client.connected() && ((currentMillis-previousMillis)<=3000)) {    // loop while the client's connected
      esp_task_wdt_reset();
      delay(1);//mandatory for wdt
      currentMillis = millis();
      if (client.available()) {     // if there's bytes to read from the client,
        char c = client.read();     // read a byte, then
        Serial.write(c);            // print it out the serial monitor
        header += c;
        if (c == '\n') {  // if the byte is a newline character
          // if the current line is blank, you got two newline characters in a row.
          // that's the end of the client HTTP request, so send a response:
          if (currentLine.length() == 0) {
            // HTTP headers always start with a response code (e.g. HTTP/1.1 200 OK)
            // and a content-type so the client knows what's coming, then a blank line:
            client.println("HTTP/1.1 200 OK");
            client.println("Content-type:text/html");
            client.println("Connection: close");
            client.println();

            if (header.indexOf("GET /LF") >= 0) {
              bMotorLinForward=true;
            } else if (header.indexOf("GET /LB") >= 0) {
              bMotorLinBackward=true;
            } else if (header.indexOf("GET /LS") >= 0) {
              bMotorLinStop=true;
            } else if (header.indexOf("GET /RF") >= 0) {
              bMotorRotForward=true;
            } else if (header.indexOf("GET /RB") >= 0) {
              bMotorRotBackward=true;
            } else if (header.indexOf("GET /RS") >= 0) {
              bMotorRotStop=true;
            } else if (header.indexOf("GET /?P=") >= 0) {
              ix = header.indexOf("GET /?P=");
              dest = header.substring(ix + 8, ix + 12);
              if (dest.indexOf("&") > 0)
              {
                dest2 = dest.substring(0, dest.indexOf("&"));
                MotorLinPerc = dest2.toInt();
                Serial.print("MotorLin go to ");
                Serial.println(MotorLinPerc);
              }
              bUpdateStatus = true;
            }
            else if (header.indexOf("GET /?LL=") >= 0) {
              ix = header.indexOf("LL=");
              dest = header.substring(ix + 3, ix + 7);
              if (dest.indexOf("&") > 0)
              {
                dest2 = dest.substring(0, dest.indexOf("&"));
                LED_BRIGHT_Val = dest2.toInt();
              }
              ix = header.indexOf("LR=");
              dest = header.substring(ix + 3, ix + 7);
              if (dest.indexOf("&") > 0)
              {
                dest2 = dest.substring(0, dest.indexOf("&"));
                LED_RED_Val = dest2.toInt();
              }
              ix = header.indexOf("LG=");
              dest = header.substring(ix + 3, ix + 7);
              if (dest.indexOf("&") > 0)
              {
                dest2 = dest.substring(0, dest.indexOf("&"));
                LED_GREEN_Val = dest2.toInt();
              }
              ix = header.indexOf("LB=");
              dest = header.substring(ix + 3, ix + 7);
              if (dest.indexOf("&") > 0)
              {
                dest2 = dest.substring(0, dest.indexOf("&"));
                LED_BLUE_Val = dest2.toInt();
    
              }
              LightLed();
              bUpdateStatus = true;
            }

            //Preserve value on reload
            client.println("<script>");
            client.println("window.onload = function() {var name = localStorage.getItem('P');if (name !== null) document.getElementById(\"P\").value=name;");
            client.println("var name = localStorage.getItem('LL');if (name !== null) document.getElementById(\"LL\").value=name;");
            client.println("var name = localStorage.getItem('LR');if (name !== null) document.getElementById(\"LR\").value=name;");
            client.println("var name = localStorage.getItem('LG');if (name !== null) document.getElementById(\"LG\").value=name;");
            client.println("var name = localStorage.getItem('LB');if (name !== null) document.getElementById(\"LB\").value=name;}");
            client.println("window.onbeforeunload = function() {localStorage.setItem('P', document.getElementById(\"P\").value);");
            client.println("localStorage.setItem('LL', document.getElementById(\"LL\").value);");
            client.println("localStorage.setItem('LR', document.getElementById(\"LR\").value);");
            client.println("localStorage.setItem('LG', document.getElementById(\"LG\").value);");
            client.println("localStorage.setItem('LB', document.getElementById(\"LB\").value);}");
            client.println("</script>");

            // Display the HTML web page
            client.println("<!DOCTYPE html><html>");
            client.println("<head><meta name=\"viewport\" content=\"width=device-width, initial-scale=1\">");
            client.println("<link rel=\"icon\" href=\"data:,\">");
            client.println("<meta http-equiv=\"refresh\" content=\"3; url=/\">");
            // CSS to style buttons
            // Feel free to change the background-color and font-size attributes to fit your preferences
            client.println("<style>html { font-family: Helvetica; display: inline-block; margin: 0px auto; text-align: center;}");
            client.println(".button { background-color: #4CAFF0; border: none; color: black; padding: 10px 20px;");
            client.println("text-decoration: none; font-size: 20px; margin: 2px; cursor: pointer;}");
            client.println(".input2 { background-color: #4CAFF0; border: none; color: black; padding: 10px 20px;");
            client.println("text-decoration: none; font-size: 20px; margin: 2px; cursor: pointer;}");
            client.println(".input3 { background-color: #FFFFFF; color: black; padding: 5px 5px; width: 58px;");
            client.println("text-decoration: none; font-size: 16px; margin: 1px; cursor: pointer;}</style></head>");

            // Web Page Heading
            client.println("<body><h1>CDROM_motors</h1>");
            client.print(FwVersion);
            client.println("<a href=\"https://www.turcotronics.it\"> www.turcotronics.it</a>");
            client.println("<a href=\"https://www.ebay.it/usr/turcotronics\"> - eBay Shop</a>");

            if (EndStopVal == 1) {
              client.println("<p><b>Backward End Stop switch reached</b></p>");
            } else if (MotorLinTime >= uiFORWARDSTOPTIME) {
              client.println("<p><b>Forward Time Stop reached</b></p>");
            } else {
              client.println("<p><b>No stop conditions</b></p>");
            }

            if (MotorLin_Run == MOTORFORWARD) {
              client.println("<p><b>Linear motor in forward</b></p>");
            } else if (MotorLin_Run == MOTORBACKWARD) {
              client.println("<p><b>Linear motor in backward</b></p>");
            } else if (MotorLin_Run == MOTORSTOP) {
              client.println("<p><b>Linear motor in stop</b></p>");
            }

            if (MotorRot_Run == MOTORFORWARD) {
              client.println("<p><b>Rotary motor in forward</b></p>");
            } else if (MotorRot_Run == MOTORBACKWARD) {
              client.println("<p><b>Rotary motor in backward</b></p>");
            } else if (MotorRot_Run == MOTORSTOP) {
              client.println("<p><b>Rotary motor in stop</b></p>");
            }

            str = String(round(MotorLinTime * 100 / uiFORWARDSTOPTIME), 0);
            client.println("<p><b>MotorLin Position: " + str + "%</b></p>");

            if(OptiSenseVal)
              client.println("<p><b>Optical sensor ON</b></p>");
            else
              client.println("<p><b>Optical sensor OFF</b></p>");

            client.println("<form action='/' method='GET'>");
            client.println("LED\% W<input class='input3' id='LL' type='number' name='LL' min=\"0\" max=\"100\" maxlength=\"3\">");
            client.println("R<input class='input3' id='LR' type='number' name='LR' min=\"0\" max=\"100\" maxlength=\"3\">");
            client.println("G<input class='input3' id='LG' type='number' name='LG' min=\"0\" max=\"100\" maxlength=\"3\">");
            client.println("B<input class='input3' id='LB' type='number' name='LB' min=\"0\" max=\"100\" maxlength=\"3\">");
            client.println("<input class='input2' type='submit' name='SUBMIT' value='Go!'></form>");

            client.println("<p>MotorLin <a href=\"/LF\"><button class=\"button\">Forward</button></a>");
            client.println("<a href=\"/LB\"><button class=\"button\">Backward</button></a>");
            client.println("<a href=\"/LS\"><button class=\"button\">Stop</button></a></p>");

            client.println("<form action='/' method='GET'>");
            sprintf(charBuffer, "Go to \%: <input class='input3' id='P' type='number' name='P' min=\"0\" max=\"100\" maxlength=\"3\">");
            client.println(charBuffer);
            client.println("<input class='input2' type='submit' name='SUBMIT' value='Go!'></form>");

            client.println("<p>MotorRot<a href=\"/RF\"><button class=\"button\">Forward</button></a>");
            client.println("<a href=\"/RB\"><button class=\"button\">Backward</button></a>");
            client.println("<a href=\"/RS\"><button class=\"button\">Stop</button></a></p>");

            client.println("</body></html>");

            // The HTTP response ends with another blank line
            client.println();
            // Break out of the while loop
            break;
          } else {  // if you got a newline, then clear currentLine
            currentLine = "";
          }
        } else if (c != '\r') {  // if you got anything else but a carriage return character,
          currentLine += c;      // add it to the end of the currentLine
        }
      }
    }
    // Clear the header variable
    header = "";
    // Close the connection
    client.stop();
    Serial.println("Client disconnected.");
    Serial.println("");
  }
  delay(1);
}
