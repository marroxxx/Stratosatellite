#include <SoftwareSerial.h>
#include <Arduino.h>


// Настраиваем SoftwareSerial на пинах D7 (RX) и D8 (TX)
SoftwareSerial loraSerial(2, 3); // RX, TX
/*
ПОДКЛЮЧЕНИЯ ARDUINO NANO к LORA:
3v3 -> VCC (или 5V)
GND -> M1 (которая со стороны 3v3)
GND -> GND
D2 -> TX
D3 -> RX
*/

void setup() {
  // Инициализация последовательного порта для отладки
  Serial.begin(9600);
  
  // Инициализация SoftwareSerial для работы с E220
  loraSerial.begin(9600); // Скорость по умолчанию 9600

  Serial.println("LoRa E220-400T22D готов к работе");
}

void loop() {
  // Отправка сообщения через LoRa модуль
  if (Serial.available()) {
    String message = Serial.readString();
    loraSerial.println(message); // Отправка данных на LoRa модуль
    Serial.print("Отправлено сообщение: ");
    
    Serial.println(message);
  }
}
