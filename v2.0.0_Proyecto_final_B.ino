#include "AdafruitIO_WiFi.h"
#include <Wire.h>
#include "Adafruit_SHT31.h"
#include <ESP32Servo.h>

Adafruit_SHT31 sensor = Adafruit_SHT31();

#include <Adafruit_NeoPixel.h>

#define PIXEL_COUNT 4
#define PIXEL_PIN1 15 
#define PIXEL_PIN2 13

Adafruit_NeoPixel pixels1(PIXEL_COUNT, PIXEL_PIN1, NEO_GRB + NEO_KHZ800);
Adafruit_NeoPixel pixels2(PIXEL_COUNT, PIXEL_PIN2, NEO_GRB + NEO_KHZ800);

#include <TB6612_ESP32.h>

//MOTOR 2
#define BIN1 25
#define BIN2 26
#define PWMB 14

#include <EasyBuzzer.h>
int buzzer = 27;

int trig=17;
int echo=16;
long t;
long dis; 

const int offsetB = 1;
Servo myservo;
int servo1 = 2;

Motor motor = Motor(BIN1, BIN2, PWMB, offsetB, 0, 5000, 8, 2);

#define IO_USERNAME  "****"
#define IO_KEY       "****"

#define WIFI_SSID      "****"
#define WIFI_PASS      "****"

AdafruitIO_WiFi io(IO_USERNAME, IO_KEY, WIFI_SSID, WIFI_PASS);


AdafruitIO_Feed *activ_intens = io.feed("aintens");
AdafruitIO_Feed *noepix_inten = io.feed("intensnpx");
AdafruitIO_Feed *luces = io.feed("luces");
AdafruitIO_Feed *hexcolor = io.feed("hexcolor");
AdafruitIO_Feed *humeIO = io.feed("hume");
AdafruitIO_Feed *tempIO = io.feed("temp");
AdafruitIO_Feed *PIR = io.feed("PIR_S");
AdafruitIO_Feed *ULTS = io.feed("UltraS");
AdafruitIO_Feed *tempLi = io.feed("limitT");
AdafruitIO_Feed *Bedroom = io.feed("Broom");


unsigned long intervalo=20000;
unsigned long previousMillis=0;

int pirS=4;


void setup() {

    Serial.begin(115200);
    Wire.begin(21, 22);

pinMode(echo, 0);
pinMode(trig, 1);

ESP32PWM::allocateTimer(0);
ESP32PWM::allocateTimer(1);
ESP32PWM::allocateTimer(2);
ESP32PWM::allocateTimer(3);
myservo.setPeriodHertz(50);
 myservo.attach(servo1, 1000, 2000);

      if(sensor.begin(0x44)){
    Serial.println("Sensor encontrado");
  }else{
    Serial.println("Sensor NO encontrado");
    while(1){
      delay(10);
    }
  }
    pinMode(pirS, INPUT);

   EasyBuzzer.setPin(buzzer);

    Serial.print("Connecting to Adafruit IO");
  io.connect();

    while(io.status() < AIO_CONNECTED) {
    Serial.print(".");
    delay(500);
  }

  Serial.println();
  Serial.println(io.statusText());

  //activa el cambio
  activ_intens->onMessage(funcion_act_neopixi);
  //slider luces
  noepix_inten->onMessage(funcion_neopixi);
  //luces encender y apagar
  luces->onMessage(funcion_luces);
  //cambio de luces
  hexcolor->onMessage(funcion_hexcolor);
  //luces pir
  PIR->onMessage(funcion_pir);
  //ultrasonico
  ULTS->onMessage(funcion_U);
  //temperatura límite
  tempLi->onMessage(funcion_limite_temp);
  //botón luces bedroom
  Bedroom->onMessage(funcion_BroomL);

}

void loop() {
    io.run();

digitalWrite(trig, 0);
delayMicroseconds(2);
digitalWrite(trig,1);
delayMicroseconds(10);
digitalWrite(trig,0);
t=pulseIn(echo,1);
dis=(t/2)/29;
Serial.print(dis);
Serial.println(" cm");
delay(500);

  float temp = sensor.readTemperature();
  float hume = sensor.readHumidity();
  int PIRL = digitalRead(pirS);
  bool val_pir;
  bool val;
  bool Tlimit;
  
 unsigned long currentMillis = millis();
  if (currentMillis - previousMillis >= intervalo) {
    //isnan -> is not a number?
  if(isnan(temp)){
    Serial.println("Error al obtener temperatura");
  }else{
    Serial.print("Temperatura *C = ");
    Serial.println(temp);
    tempIO->save(temp);
       
  }

  //! isnan -> is not not a number? -> is a number?
  if(!isnan(hume)){
    Serial.print("Humedad % = ");
    Serial.println(hume);
    humeIO->save(hume);
      
  }else{
    Serial.println("Error al obtener humedad");
  }
  
  if(PIRL==1)
  {
    val_pir=true;
    PIR->save(val_pir);
    
    }
   else{
    val_pir=false;
    PIR->save(val_pir);
    
    }

if(dis<=20)
  {
    val=true;
    ULTS->save(val);
    
    }
  else if (dis>20)
  {
    val=false;
    ULTS->save(val);
    }

if(temp>=40)
{ 
  EasyBuzzer.beep(200);
  Tlimit=true;
  tempLi->save(Tlimit);
  }
else
{
  Tlimit=false;
  tempLi->save(Tlimit);
  }

  previousMillis = currentMillis;
  }
  delay(10);
}

int flag_neopixi = 0;
int intens = 20;
void funcion_act_neopixi(AdafruitIO_Data *data)
{
  if(flag_neopixi){
   pixels1.clear();
      pixels2.clear();
      pixels1.show();
      pixels2.show();
    flag_neopixi = 0;
  }
  else{
    for(int i=0; i<PIXEL_COUNT; i++) 
    { 
pixels1.setPixelColor(i, pixels1.Color(intens, intens, intens));
pixels2.setPixelColor(i, pixels2.Color(intens, intens, intens));
    flag_neopixi=1;
    }
 }
}
 void funcion_neopixi(AdafruitIO_Data *data)
 {
     Serial.println("Cambio de intensidad   de luces");
     int x= data->toInt();
     Serial.println(x);
     intens = x;
    if(flag_neopixi){
    for(int i=0; i<PIXEL_COUNT; i++) 
    { 
    pixels1.setPixelColor(i, pixels1.Color(intens, intens, intens));
    pixels2.setPixelColor(i, pixels2.Color(intens, intens, intens));
     
      }
      pixels1.show();
      pixels2.show();
    }
      
 }

   int flag_luces = 0;
   int r = 20;
   int g = 20;
   int b = 20;

 void funcion_luces(AdafruitIO_Data *data)
 {
    Serial.println("Funcion de las Luces");
    if(flag_luces){
      pixels1.clear();
      pixels2.clear();
      pixels1.show();
      pixels2.show();
      flag_luces=0;
      }
  else{

    for(int i=0; i<PIXEL_COUNT; i++) 
    { 
       
    pixels1.setPixelColor(i, pixels1.Color(r, g, b));
    pixels2.setPixelColor(i, pixels2.Color(r, g, b));
    }
    pixels1.show();  
    pixels2.show();  
   flag_luces=1;
    } 
 }

void funcion_hexcolor(AdafruitIO_Data *data)
{
  Serial.println("Cambio de Color luces");
  String hexValue=data->toString();
  hexValue = hexValue.substring(2);
  Serial.println(hexValue);

 colorConverter(hexValue);

 if(flag_luces)
 {
  for(int i = 0; i < PIXEL_COUNT; i++)
    {
       pixels1.setPixelColor(i, pixels1.Color(r, g, b));
       pixels2.setPixelColor(i, pixels2.Color(r, g, b));
      
      }
     pixels1.show();  
     pixels2.show(); 
  }
  
  }
  

void colorConverter(String hexValue)
{
 int number = (int)strtol(&hexValue[0], NULL, 16); 
 r = number >> 16 & 0xFF;
 g = number >> 8 & 0xFF;
 b = number & 0xFF;

  Serial.print("R is");
  Serial.println(r);
  
  Serial.print("G is");
  Serial.println(g);
  
  Serial.print("B is");
  Serial.println(b);
  }

int i=0;
 void funcion_pir(AdafruitIO_Data *data)
 { 
  int PIR= data->toInt();
     Serial.println(PIR);
     i=PIR;
  if( i==1)
  {
    Serial.println("movimiento dentro");
      pixels1.clear();
      pixels2.clear();
      pixels1.show();
      pixels2.show();  
    for(int i = 0; i < PIXEL_COUNT; i++)
    {
       pixels1.setPixelColor(i, pixels1.Color(210, 222, 39));
    
      
      }
     pixels1.show();  
     pixels1.clear();

    }

    else
    {
      Serial.println("no movimiento dentro");
     pixels1.clear();
     pixels1.show(); 
      }

 }

 int o = 0;
void funcion_U(AdafruitIO_Data *data)
{
  int serv= data->toInt();
     Serial.println(serv);
     o = serv;
  if(o==1)
  {
  Serial.print("Movimiento al Frente");
  myservo.write(90);
  delay(15);
  
  }
  else if (o==0)
  {
  myservo.write(0);
  delay(15);
  Serial.print("No movimiento al frente");
  
  }
}

 int u = 0;
 void funcion_limite_temp(AdafruitIO_Data *data)
 {
  int venT= data->toInt();
     Serial.println(venT);
     u = venT;
  if(u==1)
    {
      forward(motor, motor, 130);
      }
  else
    {
     brake(motor, motor);
    }
  }

int flag_bedroom=0;
void funcion_BroomL(AdafruitIO_Data *data)
{
      pixels1.clear();
      pixels2.clear();
      pixels1.show();
      pixels2.show();
      
  if(flag_bedroom)
  { 
      pixels2.clear();
      pixels2.show();
     
   flag_luces=0;
    }
    else
    {
      for(int i=0; i<PIXEL_COUNT; i++) 
    { 
       
    pixels2.setPixelColor(i, pixels1.Color(210, 222, 39));
    }
    pixels2.show();  
    pixels2.show();  
   flag_luces=1;
      }
  
  }
