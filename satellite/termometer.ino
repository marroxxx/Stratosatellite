#include <DHT.h>

#define DHTPIN 2   // Пин, к которому подключен термометр
#define DHTTYPE DHT11   // Тип датчика - DHT11
#define HEATER_PIN 49 // Пин питания для обогревателя
#define TEMPERATURE_MIN 37
#define TEMPERATURE_MAX 38
#define DELAY_TIME 4000 //время между измерениями
#define CH1_PIN 42 //Пин питания для термометра

DHT dht(DHTPIN, DHTTYPE);

void setup() {
  Serial.begin(9600);
  dht.begin();
  pinMode(HEATER_PIN, OUTPUT);
  pinMode(CH1_PIN, OUTPUT);
}

void loop() {
  digitalWrite(CH1_PIN, HIGH);
  float humidity = dht.readHumidity();    // Считываем влажность
  float temperature = dht.readTemperature();  // Считываем температуру (в градусах Цельсия)

  delay(DELAY_TIME);
  // Проверка на ошибки при чтении данных
  
  if (isnan(humidity) || isnan(temperature)) {
    Serial.println("Ошибка чтения с датчика DHT11");
    digitalWrite(HEATER_PIN, LOW); //выключение, если термометр сломался
    return;
  }

  // Вывод данных в Serial Monitor
  Serial.print("Влажность: ");
  Serial.print(humidity);
  Serial.print(" %\t");
  Serial.print("Температура: ");
  Serial.print(temperature);
  Serial.println(" *C");

  
  if (temperature <= TEMPERATURE_MIN) {
    Serial.println("Обогреватель включён");
    digitalWrite(HEATER_PIN, HIGH); 
  } else if (temperature >= TEMPERATURE_MAX) {
    Serial.println("Обогреватель выключен");
    digitalWrite(HEATER_PIN, LOW); 
  }
  

  delay(DELAY_TIME);  // Задержка между измерениями
}