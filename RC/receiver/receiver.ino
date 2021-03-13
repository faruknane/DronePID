#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>

RF24 radio(9, 8);  // CE, CSN

const byte address[6] = "00001";

const int authcode = 5231968;

struct packet
{
  float throttle;
  float acix;
  float aciy;
  float aciz;
  int code; 
};

packet p;

void setup()
{
  while (!Serial);
    Serial.begin(250000);
  
  radio.begin();
  radio.openReadingPipe(0, address);
  radio.startListening();
  radio.setPALevel(RF24_PA_MAX);
  radio.setDataRate(RF24_250KBPS);
}

long t = 0;

void loop()
{
  
  if (radio.available())
  {
    p.code = -1;
    radio.read(&p, sizeof(p));
    if(p.code == authcode)
    {
      long pa = micros() - t;
      Serial.println(p.throttle);
      Serial.print(pa / 1000.0);
      Serial.print(" ");
      Serial.println(t);
    }
  }
    
  t = micros();
}
