#include<Wire.h>
 
const int MPU_addr=0x68;
int16_t AcX,AcY,AcZ,Tmp,GyX,GyY,GyZ;
 
int minVal=265;
int maxVal=402;
 
double x;
double y;
double z;
int count=0;

 
void setup(){
Wire.begin();
Wire.beginTransmission(MPU_addr);
Wire.write(0x6B);
Wire.write(0);
Wire.endTransmission(true);
Serial.begin(9600);
Serial.println("CLEARSHEET");
Serial.println("LABEL, frame, AngleX, AngleY,AngleZ");
}
void loop(){
Wire.beginTransmission(MPU_addr);
Wire.write(0x3B);
Wire.endTransmission(false);
Wire.requestFrom(MPU_addr,14,true);
AcX=Wire.read()<<8|Wire.read();
AcY=Wire.read()<<8|Wire.read();
AcZ=Wire.read()<<8|Wire.read();
int xAng = map(AcX,minVal,maxVal,-90,90);
int yAng = map(AcY,minVal,maxVal,-90,90);
int zAng = map(AcZ,minVal,maxVal,-90,90);
if (count<=100){

int sensorValue = analogRead(A4);
//Serial.print("frame,AngleX,AngleY,AngleZ,");
//Serial.print("DATA, count, pos1,pos2,pos3");


//Serial.print("frame");
//Serial.println(count);
//Serial.println("ms");

 
x= RAD_TO_DEG * (atan2(-yAng, -zAng)+PI);
y= RAD_TO_DEG * (atan2(-xAng, -zAng)+PI);
z= RAD_TO_DEG * (atan2(-yAng, -xAng)+PI);

int pos1 = map(x,0,180,0,180);
int pos2 = map(y,0,180,0,180);
int pos3 = map(z,0,180,0,180);

//Serial.print("AngleX= ");
//Serial.println(pos1);
 
//Serial.print("AngleY= ");
//Serial.println(pos2);
Serial.print("DATA");
Serial.print(",");
Serial.print(count);
Serial.print(",");
Serial.print(pos1);
Serial.print(",");
Serial.print(pos2);
Serial.print(",");
Serial.print(pos3);
Serial.print(",");
Serial.println(sensorValue);
//Serial.print("AngleZ= ");
//Serial.println(pos3);
//Serial.println("-----------------------------------------");
delay(200);
count++;
}
else{
  "it is fault";}
}
