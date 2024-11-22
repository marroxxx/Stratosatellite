#include <DHT.h>
#include <Wire.h>
#include <MPU6050_light.h>
#include <Arduino.h>


#define DHTPIN 2   // Пин, к которому подключен термометр
#define DHTTYPE DHT11   // Тип датчика - DHT11
#define HEATER_PIN 49 // Пин питания для обогревателя
#define TEMPERATURE_MIN 37
#define TEMPERATURE_MAX 38
#define DELAY_TIME 4000 //время между измерениями
#define CH1_PIN 42 //Пин питания для термометра
#define EEPROM_SIZE 512
#define LORA_PIN_1 8
#define LORA_PIN_2 9
#define DELAY_TIME 5000
#define LOG_INTERVAL 5000
#define ERROR_MESSAGE 10000


MPU6050 mpu(Wire); //mpu
DHT dht(DHTPIN, DHTTYPE);
String Data[3] = {"WD,", "WD,", "WD,"}; // Хранение широты, долготы и высоты
boolean getStarted = false;
String string_convert = "";
int index = 0;
boolean recievedFlag = false;
unsigned long lastLogTime = 0; 


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
  Serial.begin(9600);
  Serial1.begin(9600); //Для LoRa
  Serial2.begin(9600); // OpenLog (SD карта)
  Serial3.begin(9600); // Подключение к GPS-модулю 
  Wire.begin(); // для гироскопа

  byte status = mpu.begin();
  if (status != 0) {
    Serial.print("Ошибка инициализации MPU6050. Код ошибки: ");
    Serial.println(status);
    while (1);
  }

  Serial.println("MPU-6050 успешно инициализирован!");
  delay(1000);

  dht.begin();
  pinMode(LORA_PIN_1, OUTPUT);
  pinMode(LORA_PIN_2, OUTPUT);
  pinMode(HEATER_PIN, OUTPUT);
  pinMode(CH1_PIN, OUTPUT);
}


void loop() {
  digitalWrite(CH1_PIN, HIGH);
  String out;
  float humidity = dht.readHumidity();    // Считываем влажность
  float temperature = dht.readTemperature();  // Считываем температуру (в градусах Цельсия)

  delay(DELAY_TIME);
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
  if (isnan(mpu.getAccX()) || mpu.getAccX() == 0) {
    out += String(ERROR_MESSAGE);
  } else {
    out += String(mpu.getAccX());
  }
  out += " ";

  if (isnan(mpu.getAccY()) || mpu.getAccY() == 0) {
    out += String(ERROR_MESSAGE);
  } else {
    out += String(mpu.getAccY());
  }
  out += " ";

  if (isnan(mpu.getAccZ()) || mpu.getAccZ() == 0) {
    out += String(ERROR_MESSAGE);
  } else {
    out += String(mpu.getAccZ());
  }
  out += " ";

  if (isnan(mpu.getGyroX()) || mpu.getGyroX() == 0) {
    out += String(ERROR_MESSAGE);
  } else {
    out += String(mpu.getGyroX());
  }
  out += " ";

  if (isnan(mpu.getGyroY()) || mpu.getGyroY() == 0) {
    out += String(ERROR_MESSAGE);
  } else {
    out += String(mpu.getGyroY());
  }
  out += " ";

  if (isnan(mpu.getGyroZ()) || mpu.getGyroZ() == 0) {
    out += String(ERROR_MESSAGE);
  } else {
    out += String(mpu.getGyroZ());
  }
  out += " ";

  if (isnan(mpu.getTemp()) || mpu.getTemp() == 0) {
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

  if (temperature <= TEMPERATURE_MIN) {
    Serial.println("Обогреватель включён");
    digitalWrite(HEATER_PIN, HIGH); 
  } else if (temperature >= TEMPERATURE_MAX) {
    Serial.println("Обогреватель выключен");
    digitalWrite(HEATER_PIN, LOW); 
  }

  /*     GPS            */
  parsing();
  unsigned long currentTime = millis();
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
    if (currentTime - lastLogTime) {
      Serial.print("Записано на SD карту: ");
      Serial.println(out);
      Serial2.println(out);
      currentTime = lastLogTime;
    }
    Serial1.println(out); //в эту самую как ее там антенну
  } else {
    if (currentTime - lastLogTime) {
      Serial.print("Записано на SD карту: ");
      Serial.println(out);
      Serial2.println(out);
      currentTime = lastLogTime;
    }
    Serial1.println(out);
  }
  delay(DELAY_TIME);  // Задержка между измерениями
}