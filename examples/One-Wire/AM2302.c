#include "DHT.h"
2
DHT dht;
3
void setup()
4
{
5
Serial.begin(9600);
6
Serial.println();
7
Serial.println("Status\tHumidity (%)\tTemperature (C)\t(F)");
8
 
9
dht.setup(2); // data pin 2
10
}
11
 
12
void loop()
13
{
14
delay(dht.getMinimumSamplingPeriod());
15
float humidity = dht.getHumidity(); // ดึงค่าความชื้น
16
float temperature = dht.getTemperature(); // ดึงค่าอุณหภูมิ
17
Serial.print(dht.getStatusString());
18
Serial.print("\t");
19
Serial.print(humidity, 1);
20
Serial.print("\t\t");
21
Serial.print(temperature, 1);
22
Serial.print("\t\t");
23
Serial.println(dht.toFahrenheit(temperature), 1);
24
delay(1000);
25
}