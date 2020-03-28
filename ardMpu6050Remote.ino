

/* 
 *  Initial tries for a self balancing bot
 * Build on Lib: https://github.com/jrowberg/i2cdevlib/tree/master/Arduino/MPU6050
 */


#include "MPU6050_6Axis_MotionApps20.h" //https://github.com/jrowberg/i2cdevlib/tree/master/Arduino/MPU6050
//A4 -SDA
//A5-SCL
#include "SoftwareSerial.h"
#include "GArduinoBufUtil.h"
const int BLUEINT = 2;
const int MPUINT = 3;
SoftwareSerial BTSerial(BLUEINT,10); // blue tx, blue rx
unsigned long lastAvailableTime = millis();
unsigned long debugShow = 0;
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

#define LED_PIN 13 // (Arduino is 13, Teensy is 11, Teensy++ is 6)
bool blinkState = false;
void serprintln(String s) {
  if (Serial) Serial.println(s);    
  //for (int i = 0; i < s.length(); i++)
        //BTSerial.write(s[i]);         
}

RecBuf serBuf, blueBuf;

unsigned long lastBlueSendTime = millis();
String blueState = "INIT";

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
    mpu.setXGyroOffset(173);
    mpu.setYGyroOffset(94);
    mpu.setZGyroOffset(4);
    mpu.setXAccelOffset(370); 
    mpu.setYAccelOffset(2020); 
    mpu.setZAccelOffset(1008); 

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

    pinMode(LED_PIN, OUTPUT);
}


void loop() { 
  
  if (millis() - lastBlueSendTime > 1000) {
    lastBlueSendTime = millis();
    if (blueState == "INIT") {
      BTSerial.write("AT\r\n");
      blinkState = !blinkState;
      Serial.println("led to " + String(blinkState));
      digitalWrite(LED_PIN, blinkState);
    }else  if (blueState == "OK") {
      BTSerial.write("AT+INQ\r\n");
      blueState = "WAIT";
      blinkState = !blinkState;
      Serial.println("led to " + String(blinkState));
      digitalWrite(LED_PIN, blinkState);
    }
  }
  
  loop_bt();  
  loop_balance();
  actualStateBlueReport();
}

void loop_bt() {  
  if (BTSerial.available()){    
    int  c = BTSerial.read();
     if (blueBuf.onRecv(c)) {
       Serial.println(blueBuf.origVal);
       if (blueBuf.origVal == "OK") {
        if (blueState == "INIT")
          blueState = "OK";
       }
       if (blueBuf.origVal.startsWith("+INQ:")) {
         Serial.println("debug got inq");
         char c = blueBuf.origVal.charAt(5);
         Serial.println("number is " + String(c));
         String hex = blueBuf.origVal.substring(7);
         Serial.println("debug hex " + hex);
         if (hex == "0x5C313E2D5482") {
            Serial.println("got it");
            blueState = "CONNECTED";
            blinkState = 1;
            Serial.println("led to " + String(blinkState));
            digitalWrite(LED_PIN, 1);
            BTSerial.write(("AT+CONN"+String(c)+"\r\n").c_str());
         }
       }
       
     }
  }
  // Keep reading from Arduino Serial Monitor and send to HC-05
  if (Serial.available()){
    int c = Serial.read();
    if (serBuf.onRecv(c)) {
      if (serBuf.val == "led") {
        blinkState = !blinkState;
        Serial.println("led to " + String(blinkState));
        digitalWrite(LED_PIN, blinkState);
      }else if (serBuf.val == "blueinit") {
        blueState = "INIT";
      }else if (serBuf.cmd == "show") {
        debugShow = millis()+serBuf.val.toInt();
      }
      BTSerial.write(serBuf.val.c_str());
      BTSerial.write("\r\n");
    }
  }  
}

void loop_balance() {

  if (!dmpReady) return;
  // get current FIFO count
  fifoCount = mpu.getFIFOCount();
  mpuInterrupt = false;
  mpuIntStatus = mpu.getIntStatus();
  //Serial.println("status="+String(mpuIntStatus) + " fifo="+String(fifoCount));
     // Serial.println(String(mpuIntStatus)+" " + String(fifoCount)+"/"+String(packetSize));
  if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer))
  {      
      mpu.dmpGetQuaternion(&q, fifoBuffer); //get value for q
      mpu.dmpGetGravity(&gravity, &q); //get value for gravity
      mpu.dmpGetYawPitchRoll(ypr, &q, &gravity); //get value for ypr
      if (debugShow > millis()) {
        Serial.println(String(ypr[0]) + " " + String(ypr[1])+" " + String(ypr[2]));
      }
      
   }
   //serprintln("int="+String(mpuInterrupt) + " fifoCount=" + String(fifoCount)+"/"+String(packetSize)+" i=" +String(input)+" o=" + String(output));
}
