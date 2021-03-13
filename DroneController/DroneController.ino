#include <Wire.h>
#include "MPU6050.h"
#include <Servo.h>
#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>


template<unsigned L> class MPU6050Fusion
{

public:
  int gindex = 0;

  float Smoothgx[L], SmoothgxT = 0;
  float Smoothgy[L], SmoothgyT = 0;
  float Smoothgz[L], SmoothgzT = 0;

  float Smoothacix[L], SmoothacixT = 0;
  float Smoothaciy[L], SmoothaciyT = 0;
  float Smoothaciz[L], SmoothacizT = 0;

  float acix, aciy, aciz;
  float ax, ay, az, gx, gy, gz;
  float g;

  //float accelerationx, accelerationy, accelerationz;
  //float offsetaccx, offsetaccy, offsetaccz;

  //float velocityx = 0, velocityy = 0, velocityz = 0;
  
  
  MPU6050 mpu;
  
  inline float GetSmoothedX() __attribute__((always_inline))
  {
    return SmoothacixT / L;
  } 
  inline float GetSmoothedY() __attribute__((always_inline))
  {
    return SmoothaciyT / L;
  } 
  inline float GetSmoothedZ() __attribute__((always_inline))
  {
    return SmoothacizT / L;
  }
  
  inline float GetSmoothedGyroX() __attribute__((always_inline))
  {
    return SmoothgxT / L;
  } 
  inline float GetSmoothedGyroY() __attribute__((always_inline))
  {
    return SmoothgyT / L;
  } 
  inline float GetSmoothedGyroZ() __attribute__((always_inline))
  {
    return SmoothgzT / L;
  }
  
  inline void gnext() __attribute__((always_inline))
  {
    if(gindex == 0) gindex = L - 1;
    else gindex--;
  }

  /*inline void CalculateAcceleration() __attribute__((always_inline))
  {
    
     //float aciy = acosf(normAccel.XAxis / yercekimi) * RAD_TO_DEG - 90;
       //float acix = 90 - acosf(normAccel.YAxis / yercekimi) * RAD_TO_DEG;
      
    float x = cosf((aciy + 90) * DEG_TO_RAD) * g;
    float y = cosf((90 - acix) * DEG_TO_RAD) * g;
    accelerationy = (ax - x) / g * 9.81  - offsetaccx;
    accelerationx = (ay - y) / g * 9.81  - offsetaccy;
    accelerationz = (0) / g * 9.81 - offsetaccz;
    
  }
*/

  void Initialize()
  {
    Serial.println("Initialize MPU6050.");
    
    while(!mpu.begin(MPU6050_SCALE_250DPS, MPU6050_RANGE_4G)) 
    { Serial.println("Could not find a valid MPU6050 sensor, check wiring!"); delay(500); }

    Serial.println("Calibrating Gyro!");
    mpu.setAccelOffsetX(-686);
    mpu.setAccelOffsetY(3071);
    mpu.setAccelOffsetZ(851);
    mpu.setGyroOffsetX(85);
    mpu.setGyroOffsetY(29);
    mpu.setGyroOffsetZ(6);
    mpu.setThreshold(0);

    mpu.calibrateGyro(2000);

    for(int i = 0; i < 100; i++)
    {
      mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
      float yercekimi = sqrtf(ax*ax + ay*ay + az*az);
      g += yercekimi;
      float acceY = acosf(ax / yercekimi) * RAD_TO_DEG - 90;
      float acceX = 90 - acosf(ay / yercekimi) * RAD_TO_DEG;
      acix += acceX, aciy += acceY, aciz += 0;
    }
    g /= 100;
    acix /= 100;
    aciy /= 100;
    aciz /= 100;

    /*
    float offset1=0,offset2=0,offset3=0;
    for(int i = 0; i < 1000; i++)
    {
      mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
      CalculateAcceleration();
      offset1 += accelerationx;
      offset2 += accelerationy;
      offset3 += accelerationz;
    }

    offsetaccx = offset1 / 1000;
    offsetaccy = offset2 / 1000;
    offsetaccz = offset3 / 1000;
    */
    
  }

  void Update(float timePassed)
  {
    mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
    acix += timePassed * gx;
    aciy += timePassed * gy;
    float deltaZ = timePassed * gz;
    aciz += deltaZ;
    
    deltaZ *= DEG_TO_RAD;
    float cosdeltaZ = cosf(deltaZ);
    float sindeltaZ = sinf(deltaZ);
    float acix2 = acix * cosdeltaZ + aciy * sindeltaZ;
    float aciy2 = - acix * sindeltaZ + aciy *  cosdeltaZ;
    acix = acix2;
    aciy = aciy2;
    
    float yercekimi = sqrtf(ax*ax + ay*ay + az*az);
    float acceY = acosf(ax / yercekimi) * RAD_TO_DEG - 90;
    float acceX = 90 - acosf(ay / yercekimi) * RAD_TO_DEG;

    acix = 0.998f * acix + 0.002f * acceX;
    aciy = 0.998f * aciy  + 0.002f * acceY;

    gnext();
    
    SmoothacixT -= Smoothacix[gindex]; Smoothacix[gindex] = acix; SmoothacixT += Smoothacix[gindex]; 
    SmoothaciyT -= Smoothaciy[gindex]; Smoothaciy[gindex] = aciy; SmoothaciyT += Smoothaciy[gindex]; 
    SmoothacizT -= Smoothaciz[gindex]; Smoothaciz[gindex] = aciz; SmoothacizT += Smoothaciz[gindex]; 

    SmoothgxT -= Smoothgx[gindex]; Smoothgx[gindex] = gx; SmoothgxT += Smoothgx[gindex];
    SmoothgyT -= Smoothgy[gindex]; Smoothgy[gindex] = gy; SmoothgyT += Smoothgy[gindex];
    SmoothgzT -= Smoothgz[gindex]; Smoothgz[gindex] = gz; SmoothgzT += Smoothgz[gindex];
  
    /*CalculateAcceleration();
    velocityx += accelerationx * timePassed;
    velocityy += accelerationy * timePassed;
    velocityz += accelerationz * timePassed; */
  }
};


class PID
{
public:
  float KP, KI, KD;
  float LimitP, LimitI, LimitD;
  
  float Integral;

  PID(float k1, float k2, float k3, float l1, float l2, float l3, float integralbase = 0)
  {
    KP = k1;
    KI = k2;
    KD = k3;
    LimitP = l1;
    LimitI = l2;
    LimitD = l3;
    Integral = integralbase;
  }
  
  inline float Limit(float val, float limit) __attribute__((always_inline))
  {
    if(val > limit)
      return limit;
    else if(val < -limit)
      return -limit;
    return val;
  }

  inline float Gain(float timePassed, float aciP, float aciD, bool updateIntegral) __attribute__((always_inline))
  {
    if(updateIntegral)
      Integral = Limit(Integral + KI * timePassed * aciP, LimitI);
    return Limit(KP * aciP, LimitP) + Integral + Limit(KD * aciD, LimitD);
  }

};

MPU6050Fusion<8> mpu;

PID pidx(6, 0.1, 1.5, 100, 100, 100);
PID pidy(6, 0.1, 1.5, 100, 100, 100);
PID pidz(8, 0.0, 0.0, 50, 50, 50);

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

packet packetreceived;
packet newone;

unsigned long time = 0;
unsigned long newtime = 0;

bool start = false;

Servo motor1, motor2, motor3, motor4;

void setup() 
{
  Serial.begin(250000);

  Serial.println("Attaching Motors!");
  motor1.attach(2,1000,2000);
  motor2.attach(3,1000,2000);
  motor3.attach(4,1000,2000);
  motor4.attach(5,1000,2000);

    
  if(false)
  { 
    motor1.write(2000);
    motor2.write(2000);
    motor3.write(2000);
    motor4.write(2000);
    
    delay(5000);
    motor1.write(1000);
    motor2.write(1000);
    motor3.write(1000);
    motor4.write(1000);
    
    delay(1000);
  }
  else
  {
    motor1.write(1000);
    motor2.write(1000);
    motor3.write(1000);
    motor4.write(1000);
    delay(1000);
  }

  radio.begin();
  radio.openReadingPipe(0, address);
  radio.startListening();
  radio.setPALevel(RF24_PA_MAX);
  radio.setDataRate(RF24_250KBPS);

  packetreceived.throttle = 1000;
  packetreceived.targetacix = 0;
  packetreceived.targetaciy = 0;
  packetreceived.targetaciz = 0;

  delay(1000);
  mpu.Initialize();
  
}

float maxspeed = 1600, minspeed = 1000;

inline float throttleLimit(float x)
{
  if(x > maxspeed)
  x = maxspeed;
  else if (x < minspeed)
  x = minspeed;
  return x;
}

void RemoteControl()
{
  if (radio.available())
  {
    newone.code = -1;
    radio.read(&newone, sizeof(newone));
    if(newone.code == authcode)
    {
      packetreceived = newone;
    }
  }

}

int yazdir = 0;

void loop()
{
  yazdir++;
  newtime = micros();
  float timePassed = (newtime - time) / 1000000.0;
  time = newtime;
  
  if(!start)
  {
    start = true;
    return;
  }

  //Recieve message from remote control
  if(yazdir % 5 == 0)
  {
    RemoteControl();
  }
  
  //Calculate the angle of the drone
  mpu.Update(timePassed);

  //Calculate the PID gains
  float gainx = pidx.Gain(timePassed, (mpu.GetSmoothedX() - packetreceived.targetacix), mpu.GetSmoothedGyroX(), packetreceived.throttle > 1200);
  float gainy = pidy.Gain(timePassed, (mpu.GetSmoothedY() - packetreceived.targetaciy), mpu.GetSmoothedGyroY(), packetreceived.throttle > 1200);
  float gainz = pidz.Gain(timePassed, (mpu.GetSmoothedZ() - packetreceived.targetaciz), mpu.GetSmoothedGyroZ(), packetreceived.throttle > 1200);
  
  //Control the motors
  motor1.write(throttleLimit(packetreceived.throttle) + gainx - gainz);//2    //kötü
  motor2.write(throttleLimit(packetreceived.throttle) + gainy + gainz);    //temiz 
  motor3.write(throttleLimit(packetreceived.throttle) - gainx - gainz);//4    //kötü
  motor4.write(throttleLimit(packetreceived.throttle) - gainy + gainz); //çok kötü
 
  if(yazdir == 15)
  { 
    Serial.print(" SmoothedX:");
    Serial.print(mpu.GetSmoothedX());
    Serial.print(" SmoothedY:");
    Serial.print(mpu.GetSmoothedY());
    Serial.print(" SmoothedZ:");
    Serial.print(mpu.GetSmoothedZ()); // counter clock wise dönmesi + açı oluşturur
    Serial.print(" ekX:");
    Serial.print((gainx));
    Serial.print(" ekY:");
    Serial.print((gainy));
    Serial.print(" ekZ:");
    Serial.print((gainz));
    Serial.print(" ThrottleReceived:");
    Serial.print(packetreceived.throttle);
    Serial.print(" time:");
    Serial.print(timePassed*1000);

    Serial.print(" 1:");
    Serial.print(throttleLimit(packetreceived.throttle) + gainx - gainz);//2    //kötü
    Serial.print(" 2:");
    Serial.print(throttleLimit(packetreceived.throttle) + gainy + gainz);    //temiz 
    Serial.print(" 3:");
    Serial.print(throttleLimit(packetreceived.throttle) - gainx - gainz);//4    //kötü
    Serial.print(" 4:");
    Serial.print(throttleLimit(packetreceived.throttle) - gainy + gainz); //çok kötü

    /*
    Serial.print(" accx:");
    Serial.print(mpu.accelerationx);
    Serial.print(" accy:");
    Serial.print(mpu.accelerationy);
    Serial.print(" velox:");
    Serial.print(mpu.velocityx);
    Serial.print(" veloy:");
    Serial.print(mpu.velocityy);*/
    Serial.println();

    
    yazdir = 0;
  }
}
