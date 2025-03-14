#include <DHT.h>
#include <Wire.h>
#include <MPU6050_light.h>
#include <Arduino.h>
#include <MS5611.h>
#include <SPI.h>


#define DHTPIN 2   // Пин, к которому подключен термометр
#define DHTTYPE DHT11   // Тип датчика - DHT11
#define HEATER_PIN 48 // Пин питания для обогревателя
#define TEMPERATURE_MIN 37
#define TEMPERATURE_MAX 38
#define CH1_PIN 42 //Пин питания для термометра
#define CH2_PIN 43 //Пин питания для МАДИЗа
#define EEPROM_SIZE 512
#define LORA_PIN_1 8
#define LORA_PIN_2 9
#define LOG_INTERVAL 12000
#define ERROR_MESSAGE 10000
#define MISO_PIN 50


MPU6050 mpu(Wire); //mpu
DHT dht(DHTPIN, DHTTYPE);
String Data[3] = {"WD,", "WD,", "WD,"}; // Хранение широты, долготы и высоты
boolean getStarted = false;
String string_convert = "";
int index = 0;
boolean recievedFlag = false;
unsigned long lastLogTime = 0; 
const float HEATER_ON_THRESHOLD = 37.0;
const float HEATER_OFF_THRESHOLD = 38.0;
static bool heaterState = false;
static bool mpu_working = true;
double referencePressure; // Переменная для хранения опорного давления
MS5611 ms5611; // Создание объекта для работы с датчиком


// SPI-переменные
volatile bool spiDataReady = false;
volatile String spiMessage = "";

/* -------- Обработчик прерывания SPI -------- */
ISR(SPI_STC_vect) {
  static uint8_t spiIndex = 0;
  static char spiBuffer[64]; // Буфер для передачи данных

  if (spiDataReady) {
    if (spiIndex < spiMessage.length()) {
      SPDR = spiMessage[spiIndex++]; // Передача данных посимвольно
    } else {
      SPDR = 0; // Завершение передачи
      spiDataReady = false;
      spiIndex = 0;
    }
  } else {
    SPDR = 0; // Если данных нет, отправляем нулевой байт
  }
}


/* -------- Функция парсинга данных от GPS -------- */
void parsing() {
  if (Serial3.available() > 0) {
    char incomingByte = Serial3.read();
    //Serial.print(incomingByte);
    if (getStarted) {
      if (incomingByte != ',' && incomingByte != '\n') {
        string_convert += incomingByte;
      } else {
        if (index == 0) {
          if (string_convert != "GNGGA") {
            getStarted = false;
            recievedFlag = false;
          }
        } else if (index == 2) {
          Data[0] = (string_convert == "") ? "WD" : string_convert;
        } else if (index == 4) {
          Data[1] = (string_convert == "") ? "WD" : string_convert;
        } else if (index == 9) {
          Data[2] = (string_convert == "") ? "WD" : string_convert;
        }
        string_convert = "";
        index++;
      }
    }
    if (incomingByte == '$') {
      getStarted = true;
      index = 0;
      string_convert = "";
      recievedFlag = false;
    }
    if (incomingByte == '\n' && getStarted) {
      getStarted = false;
      recievedFlag = true;
    }
  }
}


void setup() {
  delay(3000);
  Serial.begin(9600);
  Serial1.begin(9600); //Для LoRa
  Serial2.begin(9600); //OpenLog
  if (Serial2) {
    Serial.println("OpenLog подключен!");
  } else {
    Serial.println("Openlog не подключен!"); 
  }
  Serial3.begin(9600); // Подключение к GPS-модулю 
  
  Wire.begin();

  byte status = mpu.begin();
  if (status != 0) {
    mpu_working = false;
    Serial.print("Ошибка инициализации MPU6050. Код ошибки: ");
    Serial.println(status);
  }

  Serial.println("MPU-6050 успешно инициализирован!");

  dht.begin();
  if (ms5611.begin(MS5611_HIGH_RES)) {
    Serial.println("MS5611 успешно инициализирован!"); // Запуск барометра с высоким разрешением
  } else {
    Serial.println("MS5611 не найден");
  }
  referencePressure = ms5611.readPressure(); // Чтение опорного давления

  pinMode(LORA_PIN_1, OUTPUT);
  pinMode(LORA_PIN_2, OUTPUT);
  pinMode(HEATER_PIN, OUTPUT);
  pinMode(CH1_PIN, OUTPUT);
  pinMode(CH2_PIN, OUTPUT);

  SPCR |= bit(SPE);  // Включение SPI
  SPCR |= bit(SPIE); // Включение прерываний SPI
  pinMode(MISO_PIN, OUTPUT); // MISO должен быть OUTPUT на стороне Slave
}


void loop() {
  digitalWrite(CH1_PIN, HIGH);
  digitalWrite(CH2_PIN, HIGH);
  unsigned long currentTime = millis();
  if (currentTime - lastLogTime >= LOG_INTERVAL) { //currentTime - lastLogTime >= LOG_INTERVAL
    String out;
    //digitalWrite(CH1_PIN, HIGH);
    uint32_t rawTemp = ms5611.readRawTemperature(); // Чтение сырого значения температуры
    uint32_t rawPressure = ms5611.readRawPressure(); // Чтение сырого значения давления
    double realTemperature = ms5611.readTemperature(); // Получение реальной температуры
    long realPressure = ms5611.readPressure(); // Получение реального давления
    float absoluteAltitude = ms5611.getAltitude(realPressure); // Расчет абсолютной высоты
    float relativeAltitude = ms5611.getAltitude(realPressure, referencePressure); // Расчет относительной высоты
    if (realPressure == 0) {
      out += String(ERROR_MESSAGE);
      out += " ";
      out += String(ERROR_MESSAGE);
      out += " ";
      out += String(ERROR_MESSAGE);
      out += " ";
      out += String(ERROR_MESSAGE);
      out += " ";
    } else {
      out += String(realTemperature);
      out += " ";
      out += String(realPressure);
      out += " ";
      out += String(absoluteAltitude);
      out += " ";
      out += String(relativeAltitude);
      out += " ";
    }

    float humidity = dht.readHumidity();    // Считываем влажность
    float temperature = dht.readTemperature();  // Считываем температуру (в градусах Цельсия)
    // Проверка на ошибки при чтении данных
    
    if (isnan(humidity) || isnan(temperature)) {
      Serial.println("Ошибка чтения с датчика DHT11");
      digitalWrite(HEATER_PIN, LOW); //выключение, если термометр сломался
      out += String(ERROR_MESSAGE);
      out += " ";
      out += String(ERROR_MESSAGE);
      out += " ";
      humidity = ERROR_MESSAGE;
      temperature = ERROR_MESSAGE;
    } else {
      out += String(humidity);
      out += " ";
      out += String(temperature);
      out += " ";
    }

    /*
    // Вывод данных в Serial Monitor
    Serial.print("Влажность: ");
    Serial.print(humidity);
    Serial.print(" %\t");
    Serial.print("Температура: ");
    Serial.print(temperature);
    Serial.println(" *C");
    */

    
    mpu.update();
    // Чтение данных акселерометра
    if (!mpu_working || isnan(mpu.getAccX()) || mpu.getAccX() == 0) {
      out += String(ERROR_MESSAGE);
    } else {
      out += String(mpu.getAccX());
    }
    out += " ";
    if (!mpu_working || isnan(mpu.getAccY()) || mpu.getAccY() == 0) {
      out += String(ERROR_MESSAGE);
    } else {
      out += String(mpu.getAccY());
    }
    out += " ";
    if (!mpu_working || isnan(mpu.getAccZ()) || mpu.getAccZ() == 0) {
      out += String(ERROR_MESSAGE);
    } else {
      out += String(mpu.getAccZ());
    }
    out += " ";
    if (!mpu_working || isnan(mpu.getGyroX()) || mpu.getGyroX() == 0) {
      out += String(ERROR_MESSAGE);
    } else {
      out += String(mpu.getGyroX());
    }
    out += " ";
    if (!mpu_working || isnan(mpu.getGyroY()) || mpu.getGyroY() == 0) {
      out += String(ERROR_MESSAGE);
    } else {
      out += String(mpu.getGyroY());
    }
    out += " ";
    if (!mpu_working || isnan(mpu.getGyroZ()) || mpu.getGyroZ() == 0) {
      out += String(ERROR_MESSAGE);
    } else {
      out += String(mpu.getGyroZ());
    }
    out += " ";
    if (!mpu_working || isnan(mpu.getTemp()) || mpu.getTemp() == 0) {
      out += String(ERROR_MESSAGE);
    } else {
      out += String(mpu.getTemp());
    }
    out += " ";

    /*
    Serial.print("Accel X: "); Serial.print(mpu.getAccX());
    Serial.print(" | Accel Y: "); Serial.print(mpu.getAccY());
    Serial.print(" | Accel Z: "); Serial.println(mpu.getAccZ());

    // Чтение данных гироскопа
    Serial.print("Gyro X: "); Serial.print(mpu.getGyroX());
    Serial.print(" | Gyro Y: "); Serial.print(mpu.getGyroY());
    Serial.print(" | Gyro Z: "); Serial.println(mpu.getGyroZ());

    // Температура датчика
    Serial.print("Temperature: "); Serial.print(mpu.getTemp());
    Serial.println(" °C");
    */


    if (!heaterState && temperature <= HEATER_ON_THRESHOLD) {
      Serial.println("Обогреватель включён");
      digitalWrite(HEATER_PIN, HIGH);
      heaterState = true;
    } else if (heaterState && temperature >= HEATER_OFF_THRESHOLD) {
      Serial.println("Обогреватель выключен");
      digitalWrite(HEATER_PIN, LOW);
      heaterState = false;
    }
    parsing();
    
    if (recievedFlag) {
      recievedFlag = false;
      
      String output = "!";
      if (Data[0] == "WD") {
        output += "WD,";
      } else {
        String degrees = Data[0].substring(0, 2);
        String minutes = Data[0].substring(2);
        double lat = degrees.toDouble() + (minutes.toDouble() / 60.0);
        output += String(lat, 6) + ",";
      }
      
      if (Data[1] == "WD") {
        output += "WD,";
      } else {
        String degrees = Data[1].substring(0, 3);
        String minutes = Data[1].substring(3);
        double lon = degrees.toDouble() + (minutes.toDouble() / 60.0);
        output += String(lon, 6) + ",";
      }
      
      output += Data[2];


      out += output;

      
      //if (currentTime - lastLogTime) {
        Serial.print("Записано на SD карту: ");
        Serial.println(out);
        Serial2.println(out);
        lastLogTime = currentTime;
      //}
      Serial1.println(out); //в эту самую как ее там антенну
      spiMessage = out + '\n';
      spiDataReady = true;
    } else {
      //if (currentTime - lastLogTime) {
        Serial.print("Записано на SD карту: ");
        Serial.println(out);
        Serial2.println(out);
        lastLogTime = currentTime;
      //}
      Serial1.println(out);
      spiMessage = out + '\n';
      spiDataReady = true;
    }

    lastLogTime = currentTime;
  } 
}