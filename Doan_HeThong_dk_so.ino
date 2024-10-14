#include <Arduino.h>
#include <Wire.h>
#include <math.h>
#include <LiquidCrystal_I2C.h>

double P,I,D, kp =5 , ki =0.49 , kd =0.01 , e = 0, pre_e = 0, pre_pre_e = 0, pre_u = 0, u = 0, SP ;
double T = 0.01;
double position;
double alpha, gama, beta;
String Setpointstring;
String rotarystring;
const int encoderPinA = 33;
const int encoderPinB = 32;
const int motorPin1 = 26;
const int motorPin2 = 27;
const int pwmPin = 14;
volatile long freq = 25000;
const int pwmChannel = 0;
const int resolution = 8;
int xungencoder = 0;
int thoigiantd = 0, thoigiantd1 = 0, thoigiantd2 = 0, thoigiantd3 =0 ;
int value;
double positon_motor;
LiquidCrystal_I2C lcd(0x27,16,2);

void setup() {
  Serial.begin(115200);
  Wire.begin();
  lcd.init();                    
  lcd.backlight();
  pinMode(encoderPinA, INPUT);
  pinMode(encoderPinB, INPUT);
  pinMode(motorPin1, OUTPUT);
  pinMode(motorPin2, OUTPUT);
  pinMode(pwmPin, OUTPUT);
  ledcSetup(pwmChannel, freq, resolution);
  ledcAttachPin(pwmPin, pwmChannel);
  attachInterrupt(digitalPinToInterrupt(encoderPinA), Ngatngoai, RISING);
}
void Ngatngoai() {
  // Hàm ngắt được gọi mỗi khi có xung từ encoder
  if (digitalRead(encoderPinB) == HIGH) 
  {
    xungencoder++;
  } 
  else 
  {
    xungencoder--;
  }
}

void loop() 
{
  unsigned long thoigiandem = millis();
  if (thoigiandem - thoigiantd2 >= 20) 
  {
    thoigiantd2 = thoigiandem;
    positon_motor = Position_control();

    if(positon_motor>0)
    {
      ledcWrite(pwmChannel, positon_motor);
      digitalWrite(motorPin1,LOW);
      digitalWrite(motorPin2, HIGH);
    }
    else if(positon_motor<0)
    {
      ledcWrite(pwmChannel, abs(positon_motor));
      digitalWrite(motorPin1, HIGH);
      digitalWrite(motorPin2, LOW);
    }
    else
    {
      ledcWrite(pwmChannel, 0);
      digitalWrite(motorPin1,LOW);
      digitalWrite(motorPin2,LOW);
    }
  }
      
  if (thoigiandem - thoigiantd3 >= 20) 
  {
    thoigiantd3 = thoigiandem;
    lcd.setCursor(0,0);
    lcd.print(SP);
    lcd.setCursor(0,1);
    lcd.print(position);
  }
}

int Position_control() {
  position = (xungencoder*360/330);
  e = SP - position;
  alpha = 2*T*kp + ki*T*T + 2*kd;
  beta = T*T*ki - 4*kd - 2*T*kp;
  gama = 2*kd;
  u = (alpha*e + beta*pre_e + gama*pre_pre_e + 2*T*pre_u)/(2*T);
  pre_u = u;
  pre_pre_e = pre_e;
  pre_e = e;
  if (u > 255) 
  {
    u = 255;
  }
  if (u < -255)
  {
    u = -255;
  }
  
  return u;
}