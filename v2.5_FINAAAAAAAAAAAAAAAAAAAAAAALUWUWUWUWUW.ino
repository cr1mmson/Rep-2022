#include "AdafruitIO_WiFi.h"
#include <Wire.h>
#include <SPI.h>
#include <Adafruit_Sensor.h>
#include "Adafruit_BME680.h"
#include "SI1133.h"


#define BME_SCK 13
#define BME_MISO 12
#define BME_MOSI 11
#define BME_CS 10

int ldr=18;
#define BIN1 25

#define SEALEVELPRESSURE_HPA (1013.25)


#define hPinD 19  
Adafruit_BME680 bme; // I2C
SI1133 uv = SI1133();

#define AIN1 32
#define AIN2 33

int humidity_g;
int pulse=0;
float gass;

#define IO_USERNAME  "****"
#define IO_KEY       "****"

#define WIFI_SSID      "****"
#define WIFI_PASS      "****"


unsigned long intervalo=1800000;
unsigned long previousMillis=0;

AdafruitIO_WiFi io(IO_USERNAME, IO_KEY, WIFI_SSID, WIFI_PASS);


AdafruitIO_Feed *hume = io.feed("hume");
AdafruitIO_Feed *temp = io.feed("temp");
AdafruitIO_Feed *uvinf = io.feed("uv");
AdafruitIO_Feed *gas = io.feed("gas");
AdafruitIO_Feed *motor = io.feed("motor");



void setup() {
   
    pinMode(ldr,INPUT);
    pinMode(BIN1,OUTPUT);
    
  motor->onMessage(motorM);
  
  Serial.begin(9600);
  while (!Serial);
  Serial.println(F("BME680 test"));
 
      if (!bme.begin()) {
        Serial.println("Could not find a valid BME680 sensor, check wiring!");
         while (1);
    }

      // Set up oversampling and filter initialization
      bme.setTemperatureOversampling(BME680_OS_8X);
      bme.setHumidityOversampling(BME680_OS_2X);
      bme.setPressureOversampling(BME680_OS_4X);
      bme.setIIRFilterSize(BME680_FILTER_SIZE_3);
      bme.setGasHeater(320, 150); // 320*C for 150 ms


  pinMode(hPinD,INPUT);
  pinMode(AIN1,OUTPUT);
  pinMode(AIN2,OUTPUT);

         Serial.println(" | 14CORE | Si1133 Test Code ");
         Serial.println(" Initializing ...............");
          delay(2000);
    if (! uv.begin()) {
        Serial.println("Check Serial Communication");
        while (1);
       }
  Serial.println("Si1133 Data Communication Ready");
  Serial.println("Reading .......................");
  delay(1000);


      Serial.print("Connecting to Adafruit IO");
        io.connect();

    while(io.status() < AIO_CONNECTED) {
    Serial.print(".");
    delay(500);
  }

  Serial.println();
  Serial.println(io.statusText());


}
void loop() {
if(digitalRead(ldr)==1){
    digitalWrite(BIN1,HIGH);
}
else{
  digitalWrite(BIN1,LOW);
}
  humidity_g=digitalRead(hPinD);
    io.run();
 unsigned long currentMillis = millis();
  if (currentMillis - previousMillis >= intervalo) {
    bmeI();
    uvI();

    delay(1000);

if(humidity_g==1){
pulse=1;

if(pulse==1){
 for(int i=0;i<2;i++){
     waterf();
      Serial.println(i);
      delay(700);
      
    }
     pulse=0;
  }
}
if(pulse==0){
  waternof();
  }
  previousMillis = currentMillis;
  }


  
}

void waterf(){
          digitalWrite(AIN1,HIGH); 
          digitalWrite(AIN2,LOW); 
  
}

void waternof(){ 
        digitalWrite(AIN1,LOW);
        digitalWrite(AIN2,LOW);
  
  }


void uvI(){

 uint32_t uvData = uv.readUV();
  float irData = uv.readIR();

  Serial.print("Infrared Raw Data : ");  
  Serial.println(irData,BIN);

  Serial.print("UV - Ultraviolet Raw Data : ");  
  Serial.println(uvData, BIN);

  Serial.print(" UV Fin : ");  
  Serial.println(uvData);
  uvinf->save(uvData);

  if(uvData >545){
    Serial.println(" : 14 ");
  }
  
  else
  Serial.print(" UVI : ");  
  Serial.println(0.0082* (0.00391 * uvData * uvData + uvData ));
  uv.printOut();
  delay(5000);  

}


void bmeI(){
  
if (! bme.performReading()) {
    Serial.println("Failed to perform reading :(");
    return;
  }
  gass=bme.gas_resistance /1000.0;
  Serial.print("Temperature = ");
  Serial.print(bme.temperature);
  Serial.println(" *C");
  temp->save(bme.temperature);

  Serial.print("Pressure = ");
  Serial.print(bme.pressure / 100.0);
  Serial.println(" hPa");

  Serial.print("Humidity = ");
  Serial.print(bme.humidity);
  Serial.println(" %");
  hume->save(bme.humidity);

  Serial.print("Gas = ");
  Serial.print(gass);
  Serial.println(" KOhms");
  gas->save(gass);

  Serial.print("Approx. Altitude = ");
  Serial.print(bme.readAltitude(SEALEVELPRESSURE_HPA));
  Serial.println(" m");

  Serial.println();
  delay(2000);
}

  int u=0;
void motorM(AdafruitIO_Data *data){
    int estado = data->toInt();
    u=estado;
    Serial.println(u);
    if(u==1){
      digitalWrite(AIN1,HIGH); 
      digitalWrite(AIN2,LOW);
      delay(99);
    }
else{
      digitalWrite(AIN1,LOW);
      digitalWrite(AIN2,LOW);
}
}
