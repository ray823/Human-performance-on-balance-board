#include <Wire.h>

//Declaring GYRO global variables
int gyro_x, gyro_y, gyro_z;
long acc_x, acc_y, acc_z, acc_total_vector;
int temperature;
long gyro_x_cal, gyro_y_cal, gyro_z_cal;
long loop_timer;
int lcd_loop_counter;
float angle_pitch, angle_roll;
int angle_pitch_buffer, angle_roll_buffer;
boolean set_gyro_angles;
float angle_roll_acc, angle_pitch_acc;
float angle_pitch_output, angle_roll_output;
int count=0;

void setup_mpu_6050_registers() {
  //Activate the MPU-6050
  Wire.beginTransmission(0x68);         //Start communicating with the MPU-6050
  Wire.write(0x6B);                     //Send the requested starting register
  Wire.write(0x00);                     //Set the requested starting register
  Wire.endTransmission();               //End the transmission
  //Configure the accelerometer (+/-8g)
  Wire.beginTransmission(0x68);         //Start communicating with the MPU-6050
  Wire.write(0x1C);                     //Send the requested starting register
  Wire.write(0x10);                     //Set the requested starting register
  Wire.endTransmission();               //End the transmission
  //Configure the gyro (500dps full scale)
  Wire.beginTransmission(0x68);         //Start communicating with the MPU-6050
  Wire.write(0x1B);                     //Send the requested starting register
  Wire.write(0x08);                     //Set the requested starting register
  Wire.endTransmission();               //End the transmission
  
}

void calibrate_gyro(int cal_points)
{
  // keept the gyro flat and still while calibrating
  // these oare the offset values used to calculate positiion from rest.
  Serial.println("Performing Calibration....");
  for (int cal_int = 0; cal_int < cal_points ; cal_int ++) { //Run this code cal_points times
    read_mpu_6050_data();   //Read the raw acc and gyro data from the MPU-6050
    gyro_x_cal += gyro_x;   //Add the gyro x-axis offset to the gyro_x_cal variable
    gyro_y_cal += gyro_y;   //Add the gyro y-axis offset to the gyro_y_cal variable
    gyro_z_cal += gyro_z;   //Add the gyro z-axis offset to the gyro_z_cal variable
    delay(3);               //Delay 3us to simulate the ~250Hz program loop
  }
  gyro_x_cal /= cal_points; //Divide the gyro_x_cal variable by cal_points to get the avarage offset
  gyro_y_cal /= cal_points; //Divide the gyro_y_cal variable by cal_points to get the avarage offset
  gyro_z_cal /= cal_points; //Divide the gyro_z_cal variable by cal_points to get the avarage offset

  //Serial.print("gyro_x_cal:");
  //Serial.println(gyro_x_cal);
  //Serial.print("gyro_y_cal:");
  //Serial.println(gyro_y_cal);
  //Serial.print("gyro_z_cal:");
  //Serial.println(gyro_z_cal);
}



void read_gyro()
{
  read_mpu_6050_data();                                                //Read the raw acc and gyro data from the MPU-6050

  gyro_x -= gyro_x_cal;                                                //Subtract the offset calibration value from the raw gyro_x value
  gyro_y -= gyro_y_cal;                                                //Subtract the offset calibration value from the raw gyro_y value
  gyro_z -= gyro_z_cal;                                                //Subtract the offset calibration value from the raw gyro_z value

  //Gyro angle calculations
  //0.0000611 = 1 / (250Hz / 65.5)
  angle_pitch += gyro_x * 0.0000611;                                   //Calculate the traveled pitch angle and add this to the angle_pitch variable
  angle_roll += gyro_y * 0.0000611;                                    //Calculate the traveled roll angle and add this to the angle_roll variable

  //0.000001066 = 0.0000611 * (3.142(PI) / 180degr) The Arduino sin function is in radians
  angle_pitch += angle_roll * sin(gyro_z * 0.000001066);               //If the IMU has yawed transfer the roll angle to the pitch angel
  angle_roll -= angle_pitch * sin(gyro_z * 0.000001066);               //If the IMU has yawed transfer the pitch angle to the roll angel

  //Accelerometer angle calculations
  acc_total_vector = sqrt((acc_x * acc_x) + (acc_y * acc_y) + (acc_z * acc_z)); //Calculate the total accelerometer vector
  //57.296 = 1 / (3.142 / 180) The Arduino asin function is in radians
  angle_pitch_acc = asin((float)acc_y / acc_total_vector) * 57.296;    //Calculate the pitch angle
  angle_roll_acc = asin((float)acc_x / acc_total_vector) * -57.296;    //Calculate the roll angle

  //Place the MPU-6050 spirit level and note the values in the following two lines for calibration
  angle_pitch_acc -= 0.0;                                              //Accelerometer calibration value for pitch
  angle_roll_acc -= 0.0;                                               //Accelerometer calibration value for roll

  if (set_gyro_angles) {                                               //If the IMU is already started
    angle_pitch = angle_pitch * 1.0000 ;     //Correct the drift of the gyro pitch angle with the accelerometer pitch angle
    angle_roll = angle_roll * 1.0000 ;        //Correct the drift of the gyro roll angle with the accelerometer roll angle
    //angle_pitch = angle_pitch * 0.9996 + angle_pitch_acc * 0.0004;     //Correct the drift of the gyro pitch angle with the accelerometer pitch angle
    //angle_roll = angle_roll * 0.9996 + angle_roll_acc * 0.0004;        //Correct the drift of the gyro roll angle with the accelerometer roll angle
    
  }
  else {                                                               //At first start
    angle_pitch = angle_pitch_acc;                                     //Set the gyro pitch angle equal to the accelerometer pitch angle
    angle_roll = angle_roll_acc;                                       //Set the gyro roll angle equal to the accelerometer roll angle
    set_gyro_angles = true;                                            //Set the IMU started flag
  }

  //To dampen the pitch and roll angles a complementary filter is used
  angle_pitch_output = angle_pitch_output * 0.9 + angle_pitch * 0.1;   //Take 90% of the output pitch value and add 10% of the raw pitch value
  angle_roll_output = angle_roll_output * 0.9 + angle_roll * 0.1;      //Take 90% of the output roll value and add 10% of the raw roll value

}

void read_mpu_6050_data() {                   //Subroutine for reading the raw gyro and accelerometer data
  Wire.beginTransmission(0x68);               //Start communicating with the MPU-6050
  Wire.write(0x3B);                           //Send the requested starting register
  Wire.endTransmission();                     //End the transmission
  Wire.requestFrom(0x68, 14);                 //Request 14 bytes from the MPU-6050
  while (Wire.available() < 14);              //Wait until all the bytes are received
  acc_x = Wire.read() << 8 | Wire.read();     //Add the low and high byte to the acc_x variable
  acc_y = Wire.read() << 8 | Wire.read();     //Add the low and high byte to the acc_y variable
  acc_z = Wire.read() << 8 | Wire.read();     //Add the low and high byte to the acc_z variable
  temperature = Wire.read() << 8 | Wire.read(); //Add the low and high byte to the temperature variable
  gyro_x = Wire.read() << 8 | Wire.read();      //Add the low and high byte to the gyro_x variable
  gyro_y = Wire.read() << 8 | Wire.read();      //Add the low and high byte to the gyro_y variable
  gyro_z = Wire.read() << 8 | Wire.read();      //Add the low and high byte to the gyro_z variable

}

void setup() {
  Wire.begin();                                 //Start I2C as master
  Serial.begin(9600);                          //Use only for debugging
  setup_mpu_6050_registers();                   //Setup the registers of the MPU-6050 (500dfs / +/-8g) and start the gyro
  calibrate_gyro(2000);                         //Perform Gyro Calibration
  Serial.println("CLEARSHEET");
  Serial.println("LABEL, frame, Pitch Angle, Roll Angle");
}

void loop() {
  // We are going to display the GYRO data twice a second
  // but keep reading it at full speed to keep up with smoothing the data
  int sensorValue = analogRead(A4);
  unsigned long now = millis();
  while (millis() - now <= 200)
  {
    read_gyro();
  }
  // our half second is up display angles
  if (count<=20000){
  Serial.print("DATA");
  Serial.print(",");
  Serial.print(count);
  Serial.print(",");
  //Serial.print("Pitch Angle:");
  Serial.print(angle_pitch);
  //Serial.print(" Roll Angle:");
  Serial.print(",");
  Serial.print(angle_roll);
  Serial.print(",");
  Serial.println(sensorValue);
  Serial.println("");
  count++;
}else{
  "it is fault";}
}
