#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>

RF24 radio(9, 8);  // CE, CSN

const byte address[6] = "00001";

const int authcode = 5231968;

struct packet
{
  float throttle;
  float targetacix;
  float targetaciy;
  float targetaciz;
  int otherstuff;
  int code; 
};

packet p;


void setup()
{
  Serial.begin(250000);

  radio.begin();
  radio.openWritingPipe(address);
  radio.stopListening();
  radio.setPALevel(RF24_PA_MAX);
  radio.setDataRate(RF24_250KBPS);

  p.code = authcode;
  p.targetacix = 0;
  p.targetaciy = 0;
  p.targetaciz = 0;
  p.otherstuff = 0;
  p.throttle = 1000;

  while(map(analogRead(A0), 0, 1023, 1000, 2000) > 1010)
  {
     Serial.print("baslamadi throttle:");
    Serial.print(map(analogRead(A0), 0, 1023, 1000, 2000));
    Serial.print(" targetacix:");
    Serial.print(map(analogRead(A1), 0, 1023, -50, 50)/5.0f);
    Serial.print(" targetaciy:");
    Serial.print(map(analogRead(A2), 0, 1023, -50, 50)/5.0f);
    Serial.print(" targetaciz:");
    Serial.print(map(analogRead(A3), 0, 1023, -50, 50)/5.0f);
    Serial.println();
   delay(100);
  }

  
}


long t = 0;

void loop()
{
  float pastime = 0;
  if(t != 0)
  {
    long newt = micros();
    pastime = (newt  - t)/ 1000000.0;
    t = newt;
  }
  
  p.throttle = map(analogRead(A0), 0, 1023, 1000, 2000);
  p.targetacix = map(analogRead(A1), 0, 1023, -50, 50)/5.0f;
  p.targetaciy = map(analogRead(A2), 0, 1023, -50, 50)/5.0f;
  p.targetaciz = map(analogRead(A3), 0, 1023, -50, 50)/5.0f;
  
  
  radio.write(&p, sizeof(p));
  Serial.print(" throttle:");
  Serial.print(p.throttle);
  Serial.print(" targetacix:");
  Serial.print(p.targetacix);
  Serial.print(" targetaciy:");
  Serial.print(p.targetaciy);
  Serial.print(" targetaciz:");
  Serial.print(p.targetaciz);
  Serial.println();

}
