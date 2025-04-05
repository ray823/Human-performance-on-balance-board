#include<Wire.h>
#include <SoftwareSerial.h>   // 引用程式庫
// 定義連接藍牙模組的序列埠
//SoftwareSerial BT(10, 9); // 接收腳, 傳送腳
SoftwareSerial nodeCommunication = SoftwareSerial(10, 9);
byte a = 10;
char val;  // 儲存接收資料的變數

const int MPU_addr=0x68;
int16_t AcX,AcY,AcZ,Tmp,GyX,GyY,GyZ;
 
int minVal=265;
int maxVal=402;
 
double x;
double y;
double z;
 
void setup(){
Wire.begin();
Wire.beginTransmission(MPU_addr);
Wire.write(0x6B);
Wire.write(0);
Wire.endTransmission(true);
Serial.begin(9600);

Serial.println("BT is ready!");

  // 設定藍牙模組的連線速率
  // 如果是HC-05，請改成38400
  nodeCommunication.begin(38400);
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
 
x= RAD_TO_DEG * (atan2(-yAng, -zAng)+PI);
y= RAD_TO_DEG * (atan2(-xAng, -zAng)+PI);
z= RAD_TO_DEG * (atan2(-yAng, -xAng)+PI);


int pos1 = map(x,0,180,0,180);
int pos2 = map(y,0,180,0,180);
int pos3 = map(z,0,180,0,180);

Serial.print("AngleX= ");
Serial.println(pos1);
 
Serial.print("AngleY= ");
Serial.println(pos2);
 
Serial.print("AngleZ= ");
Serial.println(pos3);
Serial.println("-----------------------------------------");
delay(200);



  }
