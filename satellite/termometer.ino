#include <DHT.h>

#define DHTPIN 5     // Пин, к которому подключен DHT11
#define DHTTYPE DHT11   // Тип датчика - DHT11
#define HEATER_PIN 9
#define TEMPERATURE_MIN 25
#define TEMPERATURE_MAX 30
#define DELAY_TIME 60000

DHT dht(DHTPIN, DHTTYPE);

void setup() {
  Serial.begin(9600);
  dht.begin();
  pinMode(HEATER_PIN, OUTPUT);
}

void loop() {
  float humidity = dht.readHumidity();    // Считываем влажность
  float temperature = dht.readTemperature();  // Считываем температуру (в градусах Цельсия)

  delay(DELAY_TIME);
  // Проверка на ошибки при чтении данных
  
  if (isnan(humidity) || isnan(temperature)) {
    Serial.println("Ошибка чтения с датчика DHT11");
    return;
  }

  // Вывод данных в Serial Monitor
  Serial.print("Влажность: ");
  Serial.print(humidity);
  Serial.print(" %\t");
  Serial.print("Температура: ");
  Serial.print(temperature);
  Serial.println(" *C");

  if (temperature < TEMPERATURE_MIN) {
    Serial.println("Обогреватель включён");
    digitalWrite(HEATER_PIN, HIGH); 
  } else if (temperature > TEMPERATURE_MAX) {
    Serial.println("Обогреватель выключен");
    digitalWrite(HEATER_PIN, LOW); 
  }

  delay(DELAY_TIME);  // Задержка между измерениями - 2 секунды
}
