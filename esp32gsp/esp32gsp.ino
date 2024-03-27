/*
    This creates two empty databases, populates values, and retrieves them back
    from the SPIFFS file 
*/

// Добавим ремарку в скетч, проба на отслеживание в репозитории
// проба на PUSH
// проба на PUSH 2

#include <stdio.h>
#include <stdlib.h>
#include <sqlite3.h>
#include <SPI.h>
#include <FS.h>
#include "SPIFFS.h"

#include "AiEsp32RotaryEncoder.h"
#include "Arduino.h"

long FREQ_MIN = 200000;                // 200kHz

//AD9833
//#define AD9833_MISO 12
#define AD9833_MOSI 13
#define AD9833_SCK  14
#define AD9833_CS   15

//INA219
#define I2C_SDA     21    // INA219 SDA
#define I2C_SCK     22    // INA219 SCK

//MCP4151
#define  MCP41x1_SCK   18 // Define SCK pin for MCP4131 or MCP4151
#define  MCP41x1_MOSI  23 // Define MOSI pin for MCP4131 or MCP4151
#define  MCP41x1_MISO  19 // Define MISO pin for MCP4131 or MCP4151
#define  MCP41x1_CS     5 // Define chipselect pin for MCP4131 or MCP4151

#include <MCP4151.h>  // https://github.com/UA6EM/MCP4151/tree/mpgsp
MCP4151 Potentiometer(MCP41x1_CS, MCP41x1_MOSI, MCP41x1_MISO, MCP41x1_SCK, 250000UL, 250000UL, SPI_MODE0);
//MCP4151 Potentiometer(MCP41x1_CS);

//--------------- Create an AD9833 object ----------------
#include <AD9833.h>  // Пробуем новую по ссылкам в README закладке
//AD9833 AD(10, 11, 13);     // SW SPI over the HW SPI pins (UNO);
//AD9833 Ad9833(AD9833_CS);  // HW SPI Defaults to 25MHz internal reference frequency
AD9833 Ad9833(AD9833_CS, AD9833_MOSI, AD9833_SCK); // SW SPI speed 250kHz


#if (defined(ESP32))
#define WIFI             // Используем модуль вайфая
#include "WiFi.h"
#include <Ticker.h>
Ticker blinks;
float blinkPeriod = 0.25;

// ***   ***   ***           Для ESP32 подключаем SPI так:
#define HSPIs             // Дисплей и AD9833 на HSPI, MCP4151 на VSPI
#if defined(HSPIs)        // для ESP32 HSPI
#define TFT_CS        15  // GP15 - CS
#define TFT_RST       16  // GP16 - RESET
#define TFT_DC        17  // GP17 - 
#define TFT_MISO      12  // GP12 - MISO (MISO, RX)
#define TFT_MOSI      13  // GP13 - SDA  (MOSI, TX)
#define TFT_SCLK      14  // GP14 - SCK

#else                     // для ESP32 VSPI (SPI0)
#define TFT_CS        5   // GP5  - CS
#define TFT_RST       20  // GP20 - RESET
#define TFT_DC        21  // GP21 - 
#define TFT_MISO      19  // GP19 - MISO (MISO, RX)
#define TFT_MOSI      23  // GP23 - SDA  (MOSI, TX)
#define TFT_SCLK      18  // GP18 - SCK
#endif

// SPI definitions and macros то MCP4151
#if defined(HSPIs) // MCP4151 на VSPI если монитор на HSPI и наоборот ;-)

#define CS_pin    5
#define MOSI_pin  23
#define MISO_pin  19
#define SCK_pin   18
#else // HSPIs

#define CS_pin    15
#define MOSI_pin  13
#define MISO_pin  12
#define SCK_pin   14
#endif
#endif


/*
connecting Rotary encoder

Rotary encoder side    MICROCONTROLLER side  
-------------------    ---------------------------------------------------------------------
CLK (A pin)            any microcontroler intput pin with interrupt -> in this example pin 32
DT (B pin)             any microcontroler intput pin with interrupt -> in this example pin 21
SW (button pin)        any microcontroler intput pin with interrupt -> in this example pin 25
GND - to microcontroler GND
VCC                    microcontroler VCC (then set ROTARY_ENCODER_VCC_PIN -1) 

***OR in case VCC pin is not free you can cheat and connect:***
VCC                    any microcontroler output pin - but set also ROTARY_ENCODER_VCC_PIN 25 
                        in this example pin 25

*/
#if defined(ESP8266)
#define ROTARY_ENCODER_A_PIN D6
#define ROTARY_ENCODER_B_PIN D5
#define ROTARY_ENCODER_BUTTON_PIN D7
#else
#define ROTARY_ENCODER_A_PIN 34
#define ROTARY_ENCODER_B_PIN 35
#define ROTARY_ENCODER_BUTTON_PIN 36
#endif
#define ROTARY_ENCODER_VCC_PIN -1 /* 27 put -1 of Rotary encoder Vcc is connected directly to 3,3V; else you can use declared output pin for powering rotary encoder */

//depending on your encoder - try 1,2 or 4 to get expected behaviour
//#define ROTARY_ENCODER_STEPS 1
//#define ROTARY_ENCODER_STEPS 2
#define ROTARY_ENCODER_STEPS 4

//instead of changing here, rather change numbers above
AiEsp32RotaryEncoder rotaryEncoder = AiEsp32RotaryEncoder(ROTARY_ENCODER_A_PIN, ROTARY_ENCODER_B_PIN, ROTARY_ENCODER_BUTTON_PIN, ROTARY_ENCODER_VCC_PIN, ROTARY_ENCODER_STEPS);

void rotary_onButtonClick()
{
  static unsigned long lastTimePressed = 0;
  //ignore multiple press in that time milliseconds
  if (millis() - lastTimePressed < 500)
  {
    return;
  }
  lastTimePressed = millis();
  Serial.print("button pressed ");
  Serial.print(millis());
  Serial.println(" milliseconds after restart");
}

void rotary_loop()
{
  //dont print anything unless value changed
  if (rotaryEncoder.encoderChanged())
  {
    Serial.print("Value: ");
    Serial.println(rotaryEncoder.readEncoder());
  }
  if (rotaryEncoder.isEncoderButtonClicked())
  {
    rotary_onButtonClick();
  }
}

void IRAM_ATTR readEncoderISR()
{
  rotaryEncoder.readEncoder_ISR();
}

/* You only need to format SPIFFS the first time you run a
   test or else use the SPIFFS plugin to create a partition
   https://github.com/me-no-dev/arduino-esp32fs-plugin */
#define FORMAT_SPIFFS_IF_FAILED true

const char* data = "Callback function called";
static int callback(void *data, int argc, char **argv, char **azColName) {
   int i;
   Serial.printf("%s: ", (const char*)data);
   for (i = 0; i<argc; i++){
       Serial.printf("%s = %s\n", azColName[i], argv[i] ? argv[i] : "NULL");
   }
   Serial.printf("\n");
   return 0;
}

int db_open(const char *filename, sqlite3 **db) {
   int rc = sqlite3_open(filename, db);
   if (rc) {
       Serial.printf("Can't open database: %s\n", sqlite3_errmsg(*db));
       return rc;
   } else {
       Serial.printf("Opened database successfully\n");
   }
   return rc;
}

char *zErrMsg = 0;
int db_exec(sqlite3 *db, const char *sql) {
   Serial.println(sql);
   long start = micros();
   int rc = sqlite3_exec(db, sql, callback, (void*)data, &zErrMsg);
   if (rc != SQLITE_OK) {
       Serial.printf("SQL error: %s\n", zErrMsg);
       sqlite3_free(zErrMsg);
   } else {
       Serial.printf("Operation done successfully\n");
   }
   Serial.print(F("Time taken:"));
   Serial.println(micros()-start);
   return rc;
}

void setup() {

   Serial.begin(115200);
   sqlite3 *db1;
   sqlite3 *db2;
   int rc;

   if (!SPIFFS.begin(FORMAT_SPIFFS_IF_FAILED)) {
       Serial.println("Failed to mount file system");
       return;
   }

   // list SPIFFS contents
   File root = SPIFFS.open("/");
   if (!root) {
       Serial.println("- failed to open directory");
       return;
   }
   if (!root.isDirectory()) {
       Serial.println(" - not a directory");
       return;
   }
   File file = root.openNextFile();
   while (file) {
       if (file.isDirectory()) {
           Serial.print("  DIR : ");
           Serial.println(file.name());
       } else {
           Serial.print("  FILE: ");
           Serial.print(file.name());
           Serial.print("\tSIZE: ");
           Serial.println(file.size());
       }
       file = root.openNextFile();
   }

   // remove existing file
   SPIFFS.remove("/test1.db");
   SPIFFS.remove("/test2.db");

   sqlite3_initialize();

   if (db_open("/spiffs/test1.db", &db1))
       return;
   if (db_open("/spiffs/test2.db", &db2))
       return;

   rc = db_exec(db1, "CREATE TABLE test1 (id INTEGER, content);");
   if (rc != SQLITE_OK) {
       sqlite3_close(db1);
       sqlite3_close(db2);
       return;
   }
   rc = db_exec(db2, "CREATE TABLE test2 (id INTEGER, content);");
   if (rc != SQLITE_OK) {
       sqlite3_close(db1);
       sqlite3_close(db2);
       return;
   }

   rc = db_exec(db1, "INSERT INTO test1 VALUES (1, 'Hello, World from test1');");
   if (rc != SQLITE_OK) {
       sqlite3_close(db1);
       sqlite3_close(db2);
       return;
   }
   rc = db_exec(db2, "INSERT INTO test2 VALUES (1, 'Hello, World from test2');");
   if (rc != SQLITE_OK) {
       sqlite3_close(db1);
       sqlite3_close(db2);
       return;
   }

   rc = db_exec(db1, "SELECT * FROM test1");
   if (rc != SQLITE_OK) {
       sqlite3_close(db1);
       sqlite3_close(db2);
       return;
   }
   rc = db_exec(db2, "SELECT * FROM test2");
   if (rc != SQLITE_OK) {
       sqlite3_close(db1);
       sqlite3_close(db2);
       return;
   }

   sqlite3_close(db1);
   sqlite3_close(db2);

  //we must initialize rotary encoder
  rotaryEncoder.begin();
  rotaryEncoder.setup(readEncoderISR);
  //set boundaries and if values should cycle or not
  //in this example we will set possible values between 0 and 1000;
  bool circleValues = false;
  rotaryEncoder.setBoundaries(0, 1000, circleValues); //minValue, maxValue, circleValues true|false (when max go to min and vice versa)

  /*Rotary acceleration introduced 25.2.2021.
   * in case range to select is huge, for example - select a value between 0 and 1000 and we want 785
   * without accelerateion you need long time to get to that number
   * Using acceleration, faster you turn, faster will the value raise.
   * For fine tuning slow down.
   */
  //rotaryEncoder.disableAcceleration(); //acceleration is now enabled by default - disable if you dont need it
  rotaryEncoder.setAcceleration(250); //or set the value - larger number = more accelearation; 0 or 1 means disabled acceleration

  // This MUST be the first command after declaring the AD9833 object
  Ad9833.begin();              // The loaded defaults are 1000 Hz SINE_WAVE using REG0
  Ad9833.reset();              // Ресет после включения питания
  const unsigned long freqSPI = 250000; 
  Ad9833.setSPIspeed(freqSPI); // Частота SPI для AD9833 установлена 4 MHz
  Ad9833.setWave(AD9833_OFF);  // Turn OFF the output
  delay(10);
  Ad9833.setWave(AD9833_SINE);  // Turn ON and freq MODE SINE the output

  // выставляем минимальную частоту для цикла определения максимального тока
  Ad9833.setFrequency((float)FREQ_MIN, 0);
}

void loop() {
    //in loop call your custom function which will process rotary encoder values
  rotary_loop();
  delay(50); //or do whatever you need to do...
}
