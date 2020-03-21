

/* 
 *  Initial tries for a self balancing bot
 * Build on Lib: https://github.com/jrowberg/i2cdevlib/tree/master/Arduino/MPU6050
 */

#include "I2Cdev.h"
#include <PID_v1.h> //From https://github.com/br3ttb/Arduino-PID-Library/blob/master/PID_v1.h
#include "MPU6050_6Axis_MotionApps20.h" //https://github.com/jrowberg/i2cdevlib/tree/master/Arduino/MPU6050
//A4 -SDA
//A5-SCL
#include "SoftwareSerial.h"
const int BLUEINT = 2;
const int MPUINT = 3;
SoftwareSerial BTSerial(BLUEINT,10); // blue tx, blue rx
const int BT_BUF_LEN=128;
char sendstr[BT_BUF_LEN+16];
int sendstrpos = 0; 
unsigned long lastAvailableTime = millis();

//A5 => SCL
//A4 => SDA
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif
MPU6050 mpu;

// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[BT_BUF_LEN]; // FIFO storage buffer, for me it is 42

// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorFloat gravity;    // [x, y, z]            gravity vector
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

 

void serprintln(String s) {
  if (Serial) Serial.println(s);    
  //for (int i = 0; i < s.length(); i++)
        //BTSerial.write(s[i]);         
}

String blueReportStr = "";
String curWorkingBlueReportStr = "";
int curWorkingBlueReportStrProg=0;
unsigned long lastBlueTime = millis();
void actualStateBlueReport() {
  if (millis() - lastBlueTime < 500) return;
  if (curWorkingBlueReportStr == "") {
    curWorkingBlueReportStr = blueReportStr;
  }
  if (curWorkingBlueReportStr == "") return;
  if (curWorkingBlueReportStrProg < curWorkingBlueReportStr.length()){
     BTSerial.write(curWorkingBlueReportStr[curWorkingBlueReportStrProg++]);
  }else {
    lastBlueTime = millis();
    BTSerial.write('\n');
     curWorkingBlueReportStr = blueReportStr;     
     curWorkingBlueReportStrProg=0;
     blueReportStr = "";
  }       
}

void blueReport(String s) {
  blueReportStr = s;  
  //for (int i = 0; i < s.length(); i++)
  //  BTSerial.write(s[i]);
  //BTSerial.write('\n');
}

volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady()
{
    mpuInterrupt = true;
}

void setup() {
    Serial.begin(115200);
    serprintln("serial initialized");
    BTSerial.begin(57600);
      
    
    #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        serprintln("wire.begin");
        //Wire.begin();
        //Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
    #endif
      
  // initialize device
    serprintln(F("Initializing I2C devices..."));
    
    mpu.initialize();

     // verify connection
    serprintln(F("Testing device connections..."));
    serprintln(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

    // load and configure the DMP
    devStatus = mpu.dmpInitialize();

    
    // supply your own gyro offsets here, scaled for min sensitivity
    mpu.setXGyroOffset(82);
    mpu.setYGyroOffset(-65);
    mpu.setZGyroOffset(-8);
    mpu.setXAccelOffset(402); 
    mpu.setYAccelOffset(838); 
    mpu.setZAccelOffset(1564); 

      // make sure it worked (returns 0 if so)
    if (devStatus == 0)
    {        
        serprintln(F("Enabling DMP..."));
        mpu.setDMPEnabled(true);

        attachInterrupt(digitalPinToInterrupt(MPUINT), dmpDataReady, RISING);
        serprintln(F("DMP ready! Waiting for first interrupt..."));
        dmpReady = true;
        packetSize = mpu.dmpGetFIFOPacketSize();
    }
    else
    {
        // ERROR!
        // 1 = initial memory load failed
        // 2 = DMP configuration updates failed
        // (if it's going to break, usually the code will be 1)
        Serial.print(F("DMP Initialization failed (code "));
        Serial.print(devStatus);
        serprintln(F(")"));
    }

}


void loop() { 
  loop_bt();  
  loop_balance();
  actualStateBlueReport();
}


char receiveStr[BT_BUF_LEN+16];
int receivePos = 0;
String curBtCmd = "";
String curBtName = "";
void loop_bt() {  
  if (BTSerial.available()){    
        int c = BTSerial.read();
        Serial.write(c);        
  }
  // Keep reading from Arduino Serial Monitor and send to HC-05
  while (Serial.available()){
    int c = Serial.read();
    Serial.write(c);
    BTSerial.write(c);
  }
}

int calcCount = 0;
void loop_balance() {

  if (!dmpReady) return;
  // get current FIFO count
  fifoCount = mpu.getFIFOCount();

     // Serial.println(String(mpuIntStatus)+" " + String(fifoCount)+"/"+String(packetSize));
  /*if ((mpuIntStatus & 0x10) || fifoCount >= packetSize)
  {
      // reset so we can continue cleanly
      mpu.resetFIFO();
      fifoCount = 0;
      serprintln("FO!");   
  }
  else if (mpuIntStatus & 0x02)
  */
  if (fifoCount)
  {
      // wait for correct available data length, should be a VERY short wait
      while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();

      // read a packet from FIFO
      mpu.getFIFOBytes(fifoBuffer, packetSize);
        
      // track FIFO count here in case there is > 1 packet available
      // (this lets us immediately read more without waiting for an interrupt)
      fifoCount -= packetSize;

      mpu.dmpGetQuaternion(&q, fifoBuffer); //get value for q
      mpu.dmpGetGravity(&gravity, &q); //get value for gravity
      mpu.dmpGetYawPitchRoll(ypr, &q, &gravity); //get value for ypr
      Serial.println(String(ypr[0]) + " " + String(ypr[1])+" " + String(ypr[2]));
   }
   //serprintln("int="+String(mpuInterrupt) + " fifoCount=" + String(fifoCount)+"/"+String(packetSize)+" i=" +String(input)+" o=" + String(output));
}
