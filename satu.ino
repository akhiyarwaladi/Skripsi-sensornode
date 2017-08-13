// untuk mengatur waktu pengiriman data lewat serial //
#include <Event.h>
#include <Timer.h>
///////////////////////////////////////////////////////

// untuk baca sensor DHT22 //
#include <Adafruit_Sensor.h>
#include <DHT.h>
#include <DHT_U.h>
/////////////////////////////

// untuk baca sensor ultrasonic //
#include <NewPing.h>
//////////////////////////////////

// inisialisasi masing-masing konfigurasi sensor //
#define DHTPIN 10
#define DHTTYPE DHT22
#define TRIGGER_PIN   4  
#define ECHO_PIN      5
#define RELAY         13
#define MAX_DISTANCE  200 
///////////////////////////////////////////////////

//////////////////////////////////////////////////
int ledPin = 12;
int powerPin = A0;
long int idalat, satu, dua, durasi, state, powerVal, n_temp, n_humid, tinggi, rel=0, intvData=1;
double volt;
int idd = 14;
unsigned long previousMillis = 0;
const long interval = 60000;
//////////////////////////////////////////////////

////// panggil objek masing-masing library ////////
Timer t;
DHT dht(DHTPIN, DHTTYPE);
NewPing sonar(TRIGGER_PIN, ECHO_PIN, MAX_DISTANCE);
///////////////////////////////////////////////////

void setup() {
  
  Serial1.begin(9600);  // XBee/UART1/pins 0 and 1
  Serial.begin(9600);   // USB
  pinMode(RELAY, OUTPUT);
  dht.begin(); 
  //t.every(intvData*60000, sendData); // atur berapa lama pengiriman data 
  
}

void loop() {

  unsigned long currentMillis = millis();

  if (currentMillis - previousMillis >= (intvData*interval)) {
    // save the last time you blinked the LED
    previousMillis = currentMillis;
    sendData();
  }
  
  if (rel==0){
    digitalWrite(RELAY,HIGH);
  }
  else{
    digitalWrite(RELAY,LOW);
  }
  
  tinggi  = sonar.ping_cm();
  n_humid = dht.readHumidity(); 
  n_temp  = dht.readTemperature();

//  Serial.print("tinggi: ");
//  Serial.println(tinggi);
//  Serial.print("humid: ");
//  Serial.println(n_humid);

  
  Serial.println(Serial1.available());
  if (Serial1.available()== 4){ 
    idalat  = Serial1.read();
    satu    = Serial1.read();
    dua     = Serial1.read();
    intvData= Serial1.read();
    Serial.print("satu: ");
    Serial.println(satu);
    Serial.print("dua: ");
    Serial.println(dua);
    if(idalat == idd){
      durasi  = (satu*255)+ dua;
      durasi  = durasi*1000;

      Serial.print("durasi: ");
      Serial.println(durasi);

      // hidupkan aktuator selama durasi //
      digitalWrite(RELAY, LOW);
      delay(durasi);
      digitalWrite(RELAY, HIGH);
      rel = 0;
      /////////////////////////////////////
      
      // kirim tanda telah selesai irigasi //
      byte bt[2];
      bt[0] = idalat;
      bt[1] = 0;
      Serial1.write(bt,2);
      ///////////////////////////////////////
    }
    while(Serial1.available()){
      Serial1.read();
    }
  }
  //////// hidupkan aktuator /////////
  if (Serial1.available() == 2){
      idalat  = Serial1.read();
      state  = Serial1.read();
      
      if(idalat == idd){
        if(state == 1){
          digitalWrite(RELAY, LOW);
          rel = 1;
        }
        else{
          digitalWrite(RELAY, HIGH);
          rel = 0;
        }
      }
      // remove all unuse available //
      while(Serial1.available()){
        Serial1.read();
      }
      // END all unuse available //
  }
  /////////////////////////////////////
  t.update();
  delay(700);
}

void sendData(){
  byte bt[4];
  bt[0] = idd;       //idalat
  bt[1] = tinggi;   //datasensor ultrasonik
  bt[2] = n_humid;  //datasensor dht22
  bt[3] = n_temp;   //datasensor dht22
  Serial1.write(bt,4);
}

