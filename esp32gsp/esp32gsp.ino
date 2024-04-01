// Генератор для катушки Мишина на контроллере ESP32

// Partition Scheme: NO OTA (2MB APP, 2MB SPIFFS)
// При компиляции, в настройках IDE оключить все уведомления
// иначе вылетит по ошибке на sqlite3
// Важно!!! Измените настройки на Вашу WIFI сеть в конфигурационном файле
// Для библиотеки TFT_eSPI приведен конфигурационный файл  для ILI9341

//                    Использование GITHUB
// 1. Клонируйте проект: git clone https://github.com/UA6EM/VESTA
// 2. Исключите конфигурационный файл из индекса:
//    git update-index --assume-unchanged esp32gsp/config.h
//  (для отмены git update-index --no-assume-unchanged your_file)
// 3. Исправьте конфигурацию в соответсвии с вашей сетью
//    Изменения в этом файле на локальном компьютере теперь
//    не попадут на GITHUB


#define WIFI   // Используем модуль вайфая
#define DEBUG  // Включить отладку

#define SECONDS(x) ((x)*1000UL)
#define MINUTES(x) (SECONDS(x) * 60UL)
#define HOURS(x) (MINUTES(x) * 60UL)
#define DAYS(x) (HOURS(x) * 24UL)
#define WEEKS(x) (DAYS(x) * 7UL)
#define ON_OFF_CASCADE_PIN 32  // Для выключения выходного каскада
#define PIN_ZUM 33
#define CORRECT_PIN A3  // Пин для внешней корректировки частоты.

#if (defined(ESP32))
#ifdef WIFI
#include <WiFi.h>
//#include <HTTPClient.h>
#include <WiFiClient.h>
#endif

#include <stdio.h>
#include <stdlib.h>
#include <sqlite3.h>

#include <SPI.h>
#include <FS.h>
#include "SPIFFS.h"

#include "AiEsp32RotaryEncoder.h"
#include "Arduino.h"
#include "config.h"
//#include "function.h"

#include <Ticker.h>
Ticker my_encoder;
float encPeriod = 0.05;

// ***   ***   ***           Для ESP32 подключаем SPI так:
#define HSPIs        // Дисплей и AD9833 на HSPI, MCP4151 на VSPI
#if defined(HSPIs)   // для ESP32 HSPI
#define TFT_CS 15    // GP15 - CS
#define TFT_RST 16   // GP16 - RESET
#define TFT_DC 17    // GP17 -
#define TFT_MISO 12  // GP12 - MISO (MISO, RX)
#define TFT_MOSI 13  // GP13 - SDA  (MOSI, TX)
#define TFT_SCLK 14  // GP14 - SCK

#else                // для ESP32 VSPI (SPI0)
#define TFT_CS 5     // GP5  - CS
#define TFT_RST 20   // GP20 - RESET
#define TFT_DC 21    // GP21 -
#define TFT_MISO 19  // GP19 - MISO (MISO, RX)
#define TFT_MOSI 23  // GP23 - SDA  (MOSI, TX)
#define TFT_SCLK 18  // GP18 - SCK
#endif

// SPI definitions and macros то MCP4151
#if defined(HSPIs)  // MCP4151 на VSPI если монитор на HSPI и наоборот ;-)

#define CS_pin 5
#define MOSI_pin 23
#define MISO_pin 19
#define SCK_pin 18
#else  // HSPIs

#define CS_pin 15
#define MOSI_pin 13
#define MISO_pin 12
#define SCK_pin 14
#endif

//AD9833
//#define AD9833_MISO 12
#define AD9833_MOSI 13
#define AD9833_SCK 14
#define AD9833_CS 15

//INA219
#define I2C_SDA 21  // INA219 SDA
#define I2C_SCK 22  // INA219 SCK

//MCP4151
#define MCP41x1_SCK 18   // Define SCK pin for MCP4131 or MCP4151
#define MCP41x1_MOSI 23  // Define MOSI pin for MCP4131 or MCP4151
#define MCP41x1_MISO 19  // Define MISO pin for MCP4131 or MCP4151
#define MCP41x1_CS 5     // Define chipselect pin for MCP4131 or MCP4151

//ROTARY ENCODER
#define ROTARY_ENCODER_A_PIN 34
#define ROTARY_ENCODER_B_PIN 35
#define ROTARY_ENCODER_BUTTON_PIN 36
#define ROTARY_ENCODER_VCC_PIN -1 /* 27 put -1 of Rotary encoder Vcc is connected directly to 3,3V; else you can use declared output pin for powering rotary encoder */

//depending on your encoder - try 1,2 or 4 to get expected behaviour
#define ROTARY_ENCODER_STEPS 1
//#define ROTARY_ENCODER_STEPS 2
//#define ROTARY_ENCODER_STEPS 4

#else ECHO "Проект под микроконтроллер архитектуры ESP32";
#endif


// Глобальные переменные
unsigned long /*uint32_t*/ interval = MINUTES(1);
unsigned long /*uint32_t*/ oneMinute = MINUTES(1);
unsigned long /*uint32_t*/ timers = MINUTES(5);  // время таймера 15, 30, 45 или 60 минут
unsigned long /*uint32_t*/ memTimers = 0;        // здесь будем хранить установленное время таймера
unsigned long /*uint32_t*/ oldmemTimers = 0;
bool isWorkStarted = false;  // флаг запуска таймера

unsigned long /*uint32_t*/ timMillis = 0;
unsigned long /*uint32_t*/ oldMillis = 0;
unsigned long /*uint32_t*/ mill;  // переменная под millis()
unsigned long /*uint32_t*/ prevCorrectTime = 0;
unsigned long /*uint32_t*/ prevReadAnalogTime = 0;  // для отсчета 10 секунд между подстройкой частоты
unsigned long /*uint32_t*/ prevUpdateDataIna = 0;   // для перерыва между обновлениями данных ina
unsigned int wiperValue;                            // variable to hold wipervalue for MCP4131 or MCP4151
unsigned int Data_ina219 = 0;
long /*int32_t*/ FREQ_MIN = 200000;  // 200kHz
long /*int32_t*/ FREQ_MAX = 500000;  // 500kHz
long /*int32_t*/ ifreq = FREQ_MIN;
long /*int32_t*/ freq = FREQ_MIN;
const unsigned long /*uint32_t*/ freqSPI = 250000;  // Частота только для HW SPI AD9833
// UNO SW SPI = 250kHz
const unsigned long /*uint32_t*/ availableTimers[] = { oneMinute * 15, oneMinute * 30, oneMinute * 45, oneMinute * 60 };
const int /*uint8_t*/ maxTimers = 4;
int timerPosition = 0;
volatile int newEncoderPos;        // Новая позиция энкодера
static int currentEncoderPos = 0;  // Текущая позиция энкодера
volatile int d_resis = 127;        // Средняя позиция потенциометра
unsigned long drawTime = 0;        // Для тестирования дисплея

// Дисплей TFT 240x320 ILI9341 или аналогичный, без разницы, без TOUCH
#include <TFT_eSPI.h>
TFT_eSPI tft = TFT_eSPI();


#include <MCP4151.h>  // https://github.com/UA6EM/MCP4151/tree/mpgsp
MCP4151 Potentiometer(MCP41x1_CS, MCP41x1_MOSI, MCP41x1_MISO, MCP41x1_SCK, 250000UL, 250000UL, SPI_MODE0);
//MCP4151 Potentiometer(MCP41x1_CS);
// по умолчанию 50% потенциометра
int currentPotenciometrPercent = 127;


#include "INA219.h"
INA219 ina219;


//--------------- Create an AD9833 object ----------------
#include <AD9833.h>  // Пробуем новую по ссылкам в README закладке
//AD9833 AD(10, 11, 13);     // SW SPI over the HW SPI pins (UNO);
//AD9833 Ad9833(AD9833_CS);  // HW SPI Defaults to 25MHz internal reference frequency
AD9833 Ad9833(AD9833_CS, AD9833_MOSI, AD9833_SCK);  // SW SPI speed 250kHz

/*
  connecting Rotary encoder
  Rotary encoder side    MICROCONTROLLER side
  -------------------    ---------------------------------------------------------------------
  CLK (A pin)            any microcontroler intput pin with interrupt -> in this example pin 34
  DT (B pin)             any microcontroler intput pin with interrupt -> in this example pin 35
  SW (button pin)        any microcontroler intput pin with interrupt -> in this example pin 36
  GND - to microcontroler GND
  VCC                    microcontroler VCC (then set ROTARY_ENCODER_VCC_PIN -1)
***OR in case VCC pin is not free you can cheat and connect:***
  VCC                    any microcontroler output pin - but set also ROTARY_ENCODER_VCC_PIN 25
                         in this example pin 25
*/

//instead of changing here, rather change numbers above
AiEsp32RotaryEncoder rotaryEncoder = AiEsp32RotaryEncoder(ROTARY_ENCODER_A_PIN, ROTARY_ENCODER_B_PIN, ROTARY_ENCODER_BUTTON_PIN, ROTARY_ENCODER_VCC_PIN, ROTARY_ENCODER_STEPS);

void rotary_onButtonClick() {
  Serial.print("maxTimers = ");
  Serial.println(maxTimers);
  static unsigned long lastTimePressed = 0;
  //ignore multiple press in that time milliseconds
  if (millis() - lastTimePressed < 500) {
    return;
  }
  lastTimePressed = millis();
  Serial.print("button pressed ");
  Serial.print(millis());
  Serial.println(" milliseconds after restart");
}

void rotary_loop() {
  //dont print anything unless value changed
  if (rotaryEncoder.encoderChanged()) {
    Serial.print("Value: ");
    Serial.println(rotaryEncoder.readEncoder());
  }
  if (rotaryEncoder.isEncoderButtonClicked()) {
    rotary_onButtonClick();
  }
}

void IRAM_ATTR readEncoderISR() {
  rotaryEncoder.readEncoder_ISR();
}


/* You only need to format SPIFFS the first time you run a
   test or else use the SPIFFS plugin to create a partition
   https://github.com/me-no-dev/arduino-esp32fs-plugin */
#define FORMAT_SPIFFS_IF_FAILED true

const char *data = "Callback function called";
static int callback(void *data, int argc, char **argv, char **azColName) {
  int i;
  Serial.printf("%s: ", (const char *)data);
  for (i = 0; i < argc; i++) {
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
  int rc = sqlite3_exec(db, sql, callback, (void *)data, &zErrMsg);
  if (rc != SQLITE_OK) {
    Serial.printf("SQL error: %s\n", zErrMsg);
    sqlite3_free(zErrMsg);
  } else {
    Serial.printf("Operation done successfully\n");
  }
  Serial.print(F("Time taken:"));
  Serial.println(micros() - start);
  return rc;
}


// ФУНКЦИИ ОТ СТАРОГО СКЕТЧА БУДУТ ТУТ
// функция выбора времени работы
void setTimer() {
  // если энкодер крутим по часовой
  if (newEncoderPos - currentEncoderPos > 0) {
    if (timerPosition == maxTimers - 1) {
      timerPosition = 0;
    } else {
      timerPosition += 1;
    }
  } else if (newEncoderPos - currentEncoderPos < 0) {
    // если энкодер крутим против часовой
    if (timerPosition == 0) {
      timerPosition = maxTimers - 1;
    } else {
      timerPosition -= 1;
    }
  }
  memTimers = availableTimers[timerPosition];
}


void resetPotenciometer() {
  // Понижаем сопротивление до 0%:
  wiperValue = 1;
  Potentiometer.writeValue(wiperValue);  // Set MCP4151 to position 1
}

// Уровень percent - от 0 до 100% от максимума.
void setResistance(int percent) {
  // resetPotenciometer();
  // for (int i = 0; i < percent; i++) {
  wiperValue = percent;
  ;
  // }
  Potentiometer.writeValue(wiperValue);  // Set MCP4151
}

void processPotenciometr() {
  // если энкодер крутим по часовой
  if (newEncoderPos - currentEncoderPos > 0) {
    if (currentPotenciometrPercent >= d_resis) {
      currentPotenciometrPercent = d_resis;
      wiperValue = d_resis;
    } else {
      currentPotenciometrPercent += 1;
      wiperValue += 1;
    }
  } else if (newEncoderPos - currentEncoderPos < 0) {
    // если энкодер крутим против часовой
    if (currentPotenciometrPercent <= 1) {
      currentPotenciometrPercent = 1;
      wiperValue = 1;
    } else {
      currentPotenciometrPercent -= 1;
      wiperValue -= 1;
    }
  }
  setResistance(currentPotenciometrPercent);
  Potentiometer.writeValue(wiperValue);  // Set MCP4131 to mid position
  Serial.print("wiperValue = ");
  Serial.println(wiperValue);
}

/********* Таймер обратного отсчёта экспозиции **********/
uint32_t setTimerLCD(uint32_t timlcd) {
  if (millis() - timMillis >= 1000) {
    timlcd = timlcd - 1000;
    timMillis += 1000;
  }
  if (timlcd == 0) {
    timlcd = oldmemTimers;
    isWorkStarted = false;

    digitalWrite(ON_OFF_CASCADE_PIN, LOW);
    start_Buzzer();
    delay(3000);
    stop_Buzzer();
  }
  return timlcd;
}

/*******************ПИЩАЛКА ********************/
void start_Buzzer() {
  digitalWrite(PIN_ZUM, HIGH);
}

void stop_Buzzer() {
  digitalWrite(PIN_ZUM, LOW);
}

// ******************* Обработка AD9833 ***********************
void /*long*/ readAnalogAndSetFreqInSetup() {
  int maxValue = 0;
  long freqWithMaxI = FREQ_MIN;
  long freqIncrease = 1000;                                   // 1kHz
  int iterations = (FREQ_MAX - FREQ_MIN) / freqIncrease - 1;  // (500000 - 200000) / 1000 - 1 = 199

  for (int j = 1; j <= iterations; j++) {
    // читаем значение аналогового входа
    int tempValue = analogRead(CORRECT_PIN);
    // если значение тока больше предыдущего, запоминаем это значение и текущую частоту
    if (tempValue > maxValue) {
      maxValue = tempValue;
      freqWithMaxI = freq;
    }
    // увеличиваем частоту для дальнейшего измерения тока
    freq = freq + freqIncrease;
    if (freq > FREQ_MAX) {
      freq = FREQ_MAX;
    }
    // подаём частоту на генератор
    Ad9833.setFrequency((float)freq, 0);
    delay(20);
  }
  ifreq = freqWithMaxI;
  // подаём частоту на генератор
  Ad9833.setFrequency((float)ifreq, 0);
  prevReadAnalogTime = millis();
}

/**** Подстройка частоты каждые 1-10 секунд относительно аналогового сигнала ***/
void readAnalogAndSetFreqInLoop() {
  uint32_t curr = millis();

  // если прошло N секунд с момента последней проверки
  if (curr - prevReadAnalogTime > 1000 * 5) {  //выбор времени изменения частоты.1-10 сек.
    long availableDiff = 5000;                 // 1kHz-10kHz разница частот
    long freqIncrease = 500;                   // 100Hz-1kHz шаг увеличения частоты при сканировании

    int iterations = (availableDiff * 2) / freqIncrease - 1;  // (10000 * 2) / 1000 - 1 = 19

    long minimalFreq = ifreq - availableDiff;
    if (minimalFreq < FREQ_MIN) {
      minimalFreq = FREQ_MIN;
    }
    // подаём на генератор минимальную частоту из диапазона +-10кГц
    Ad9833.setFrequency((float)minimalFreq, 0);
    delay(20);

    int maxValue = 0;
    long freqWithMaxI = minimalFreq;
    freq = minimalFreq;

    for (int j = 1; j <= iterations; j++) {
      // читаем значение аналогового входа
      int tempValue = analogRead(CORRECT_PIN);
      // если значение тока больше предыдущего, запоминаем это значение и текущую частоту
      if (tempValue > maxValue) {
        maxValue = tempValue;
        freqWithMaxI = freq;
      }
      // увеличиваем частоту для дальнейшего измерения тока
      freq = freq + freqIncrease;
      if (freq > FREQ_MAX) {
        freq = FREQ_MAX;
      }
      // подаём частоту на генератор
      Ad9833.setFrequency((float)freq, 0);
      delay(10);
    }
    ifreq = freqWithMaxI;
    Ad9833.setFrequency((float)ifreq, 0);
    prevReadAnalogTime = millis();
  }
}

// *** Вывод на дисплей ***
void myDisplay() {
  yield();
  Serial.println("TFT Dispay is OK");
}


//  ***** ТЕСТИРОВАНИЕ БАЗЫ SQLite3 *****
void testSqlite3() {
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
  yield();
  File file = root.openNextFile();
  yield();
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

  yield();
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
  yield();
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
  yield();
  sqlite3_close(db1);
  sqlite3_close(db2);
}


/*********************** S E T U P ***********************/
void setup() {
  Serial.begin(115200);

  // Подключаемся к сети WIFI
#ifdef WIFI
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);

  //    WiFi.connect();
  while (WiFi.status() != WL_CONNECTED) {
    Serial.print(".^.");
    delay(100);
  }
#endif
  Serial.println(" Connected");
  Serial.println();

  // Дисплей
  tft.init();
  tft.setRotation(3);  // разъём слева от меня (1 - справа)
  tft.fillScreen(TFT_BLACK);
  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  Serial.println("Start TEST Grafic Display");
  Serial.println();
  testDisplay();
  Serial.println("TEST Grafic Display is OK");
  Serial.println();

  // сбрасываем потенциометр в 0%
  resetPotenciometer();                       // после сброса устанавливаем значение по умолчанию
  setResistance(currentPotenciometrPercent);  // ждем секунду после настройки потенциометра
  delay(1000);

  pinMode(ON_OFF_CASCADE_PIN, OUTPUT);
  pinMode(PIN_ZUM, OUTPUT);
  pinMode(CORRECT_PIN, INPUT);

  digitalWrite(PIN_ZUM, LOW);
  digitalWrite(ON_OFF_CASCADE_PIN, HIGH);

  // analogReference(INTERNAL);

  ina219.begin(0x40);                 // (44) i2c address 64=0x40 68=0х44 исправлять и в ina219.h одновременно
  ina219.configure(0, 2, 12, 12, 7);  // 16S -8.51ms
  ina219.calibrate(0.100, 0.32, 16, 3.2);

  SPI.begin();
  // This MUST be the first command after declaring the AD9833 object
  Ad9833.begin();               // The loaded defaults are 1000 Hz SINE_WAVE using REG0
  Ad9833.reset();               // Ресет после включения питания
  Ad9833.setSPIspeed(freqSPI);  // Частота SPI для AD9833 установлена 4 MHz
  Ad9833.setWave(AD9833_OFF);   // Turn OFF the output
  delay(10);

#ifdef DEBUG
  // тест AD9833
  Ad9833.setWave(AD9833_SQUARE2);
  testFreq(10000, 12000000, 1000);
  Ad9833.setWave(AD9833_TRIANGLE);
  testFreq(10000, 12000000, 1000);
  Ad9833.setWave(AD9833_SQUARE1);
  testFreq(10000, 12000000, 1000);
  Ad9833.setWave(AD9833_SQUARE2);
  Ad9833.setWave(AD9833_SINE);  // Turn ON and freq MODE SINE the output
  testFreq(10000, 12000000, 1000);
  // END тест AD9833
#endif

  Ad9833.setWave(AD9833_SINE);  // Turn ON and freq MODE SINE the output
  // выставляем минимальную частоту для цикла определения максимального тока
  Ad9833.setFrequency((float)FREQ_MIN, 0);

  Serial.print("freq=");
  Serial.println(FREQ_MIN);
  Serial.println();

  // Настраиваем частоту под катушку
  readAnalogAndSetFreqInSetup();
  Serial.println("Set Freq - end");
  Serial.println();

  Data_ina219 = ina219.shuntCurrent() * 1000;
  Serial.println("Data_ina219 - end");
  Serial.println();

  myDisplay();
  delay(1000);
  Serial.println("myDisplay - end");
  Serial.println();

  testSqlite3();

  //we must initialize rotary encoder
  rotaryEncoder.begin();
  rotaryEncoder.setup(readEncoderISR);
  //set boundaries and if values should cycle or not
  //in this example we will set possible values between 0 and 1000;
  bool circleValues = false;
  rotaryEncoder.setBoundaries(0, 1000, circleValues);  //minValue, maxValue, circleValues true|false (when max go to min and vice versa)

  /*Rotary acceleration introduced 25.2.2021.
     in case range to select is huge, for example - select a value between 0 and 1000 and we want 785
     without accelerateion you need long time to get to that number
     Using acceleration, faster you turn, faster will the value raise.
     For fine tuning slow down.
  */
  //rotaryEncoder.disableAcceleration(); //acceleration is now enabled by default - disable if you dont need it
  rotaryEncoder.setAcceleration(250);  //or set the value - larger number = more accelearation; 0 or 1 means disabled acceleration

  memTimers = availableTimers[0];  // выставляем 15 минут по умолчанию
#ifdef DEBUG
  testMCP4151();
#endif
  wiperValue = d_resis / 2;
  //currentEncoderPos = wiperValue;
  Potentiometer.writeValue(wiperValue);  // Set MCP4131 or MCP4151 to mid position

  //Энкодер зацепим за тикер
  my_encoder.attach(encPeriod, rotary_loop);

  Serial.println("END SETUP");
} /******************** E N D   S E T U P *******************/


/************************ L O O P ***************************/
void loop() {
  //in loop call your custom function which will process rotary encoder values
  //rotary_loop();
  delay(50);  //or do whatever you need to do...

} /******************** E N D   L O O P *******************/
