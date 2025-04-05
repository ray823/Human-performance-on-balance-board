#include "Wire.h"

// I2Cdev and MPU6050 must be installed as libraries, or else the .cpp/.h files
// for both classes must be in the include path of your project
#include "I2Cdev.h"
#include "MPU6050.h"

// class default I2C address is 0x68
// specific I2C addresses may be passed as a parameter here
// AD0 low = 0x68 (default for InvenSense evaluation board)
// AD0 high = 0x69
MPU6050 accelgyro;

int16_t ax, ay, az;
int16_t gx, gy, gz;
double total_angle=0;
#define LED_PIN 13

/* 把mpu6050放在水平桌面上，分别读取读取2000次，然后求平均值 */
#define AX_ZERO (-1476) /* 加速度计的0偏修正值 */
#define GX_ZERO (-30.5) /* 陀螺仪的0偏修正值 */

void setup() {
    // join I2C bus (I2Cdev library doesn't do this automatically)
    Wire.begin();

    // initialize serial communication
    // (38400 chosen because it works as well at 8MHz as it does at 16MHz, but
    // it's really up to you depending on your project)
    Serial.begin(38400);

    // initialize device
    Serial.println("Initializing I2C devices...");
    accelgyro.initialize();

    // verify connection
    Serial.println("Testing device connections...");
    Serial.println(accelgyro.testConnection() ? "MPU6050 connection successful" : "MPU6050 connection failed");
}

/* 通过卡尔曼滤波得到的最终角度 */
float Angle=0.0;

/*由角速度计算的倾斜角度 */
float Angle_gy=0.0;

float Q_angle=0.9;  
float Q_gyro=0.001;
float R_angle=0.5;
float dt=0.01;        /* dt为kalman滤波器采样时间; */
char  C_0 = 1;
float Q_bias, Angle_err;
float PCt_0=0.0, PCt_1=0.0, E=0.0;
float K_0=0.0, K_1=0.0, t_0=0.0, t_1=0.0;
float Pdot[4] ={0,0,0,0};
float PP[2][2] = { { 1, 0 },{ 0, 1 } };

/* 卡尔曼滤波函数,具体实现可以参考网上资料,也可以使用其它滤波算法 */
void Kalman_Filter(float Accel,float Gyro)               
{
        Angle+=(Gyro - Q_bias) * dt;

        Pdot[0]=Q_angle - PP[0][1] - PP[1][0];

        Pdot[1]=- PP[1][1];
        Pdot[2]=- PP[1][1];
        Pdot[3]=Q_gyro;
        
        PP[0][0] += Pdot[0] * dt;   
        PP[0][1] += Pdot[1] * dt;   
        PP[1][0] += Pdot[2] * dt;
        PP[1][1] += Pdot[3] * dt;
               
        Angle_err = Accel - Angle;
        
        PCt_0 = C_0 * PP[0][0];
        PCt_1 = C_0 * PP[1][0];
        
        E = R_angle + C_0 * PCt_0;
        
        if(E!=0)
        {
          K_0 = PCt_0 / E;
          K_1 = PCt_1 / E;
        }
        
        t_0 = PCt_0;
        t_1 = C_0 * PP[0][1];

        PP[0][0] -= K_0 * t_0;
        PP[0][1] -= K_0 * t_1;
        PP[1][0] -= K_1 * t_0;
        PP[1][1] -= K_1 * t_1;
               
        Angle        += K_0 * Angle_err;
        Q_bias        += K_1 * Angle_err;
}

void loop() {
    // read raw accel/gyro measurements from device

    double ax_angle=0.0;
    double gx_angle=0.0;
    unsigned long time=0;
    unsigned long mictime=0;
    static unsigned long pretime=0;
    float gyro=0.0;
    if(pretime==0)
    {
        pretime=millis();
        return;
    }
    mictime=millis();

    accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

/* 加速度量程范围设置2g 16384 LSB/g
* 计算公式：
* 前边已经推导过这里再列出来一次
* x是小车倾斜的角度,y是加速度计读出的值
* sinx = 0.92*3.14*x/180 = y/16384
* x=180*y/(0.92*3.14*16384)=
*/
   
    ax -= AX_ZERO;
ax_angle=ax/262;
   
/* 陀螺仪量程范围设置250 131 LSB//s
* 陀螺仪角度计算公式:
* 小车倾斜角度是gx_angle,陀螺仪读数是y,时间是dt
* gx_angle +=(y/(131*1000))*dt
*/
    gy -= GX_ZERO;
    time=mictime-pretime;

    gyro=gy/131.0;
    gx_angle=gyro*time;
gx_angle=gx_angle/1000.0;
total_angle-=gx_angle;
   
    dt=time/1000.0;
    Kalman_Filter(ax_angle,gyro);

    Serial.print(ax_angle); Serial.print(",");
    Serial.print(total_angle); Serial.print(",");
    Serial.println(Angle);

    pretime=mictime;
}
