

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
byte debugShow = 0;

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
unsigned long ledBlinkTime = 3000;
// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorFloat gravity;    // [x, y, z]            gravity vector
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector
float yprOld[3] = {0,0,0};
float yprMax[3] = {0,0,0}, yprMin[3]= {0,0,0};

#define LED_PIN 13 // (Arduino is 13, Teensy is 11, Teensy++ is 6)
int START_PIN = 8;

bool blinkState = false;
void serprintln(String s) {
  if (Serial) Serial.println(s);    
  //for (int i = 0; i < s.length(); i++)
        //BTSerial.write(s[i]);         
}

class QueueCommand {
  unsigned long lastCmdSendTime = millis();
  unsigned long DELAY = 200;
  bool haveCmdToSend = false;
  String curCmd = "";
  String oldCmd = "";
 public:
  QueueCommand(){
    DELAY = 200;
  }
  QueueCommand(unsigned long delay){
    DELAY = delay;
  }
  void AddCommand(String cmd) {
    haveCmdToSend = true;
    curCmd = cmd;
  }
  String GetCommand() {
    if (!haveCmdToSend) return "";
    if (millis() - lastCmdSendTime < DELAY) return ""; 
    if (curCmd == oldCmd) {
      haveCmdToSend = false;
      return "";
    }
    oldCmd = curCmd;
    lastCmdSendTime = millis();
    return curCmd;
  }
};

RecBuf serBuf, blueBuf;

QueueCommand lCmd, rCmd, genCmd;

QueueCommand* queueCmds[3];

unsigned long lastBlueSendTime = millis();
unsigned long STARTTIME = millis();
String blueState = "INIT";


volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady()
{
    mpuInterrupt = true;
}

void setup() {
  queueCmds[0] = &lCmd;
  queueCmds[1] = &rCmd;
  queueCmds[2] = &genCmd;
  
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
    pinMode(START_PIN, INPUT);
    STARTTIME = millis();
}

int oldStartPinVal = 110;
byte CANDSENDCMD = 0;

void loop() { 
  if (millis() - STARTTIME > 5000) {
    if (!CANDSENDCMD) {
       CANDSENDCMD = 1;
       Serial.println("5s passed");
    }
  }
  if (millis() - lastBlueSendTime > ledBlinkTime) {
    blinkState = !blinkState;
    digitalWrite(LED_PIN, blinkState);
    lastBlueSendTime = millis();
    if (blueState == "INIT") {
      BTSerial.write("AT\r\n");
      blueState = "WAITOK";
      Serial.println("Sent AT");
    }else  if (blueState == "OK") {
      BTSerial.write("AT+INQ\r\n");
      ledBlinkTime = 1000;
      Serial.println("Sent INQ state=" + blueState);
      blueState = "WAIT";      
    }
  }
  
  loop_bt();  
  loop_balance();

  int startPinVal = digitalRead(START_PIN);
  if (oldStartPinVal != startPinVal) {
    oldStartPinVal = startPinVal;
    Serial.println("Startpin " + String(oldStartPinVal));
  }
}

void loop_bt() {  
  if (BTSerial.available()){    
    int  c = BTSerial.read();
     if (blueBuf.onRecv(c)) {
       Serial.println(blueBuf.origVal);
       if (blueBuf.origVal == "OK") {
          if (blueState == "INIT" || blueState == "WAITOK") {
            ledBlinkTime = 1500;
            blueState = "OK";
          }
       }
       if (blueBuf.origVal.startsWith("+INQ:")) {
         Serial.println("got inq: " + blueBuf.origVal);
         char c = blueBuf.origVal.charAt(5);
         String hex = blueBuf.origVal.substring(7);
         if (hex == "0x5C313E2D5482") {
            ledBlinkTime = 500;
            Serial.println("got address, ready to connect");
            blueState = "CONNECTING";
            BTSerial.write(("AT+CONN"+String(c)+"\r\n").c_str());
         }
       }else if (blueBuf.origVal == "+Connected") {
          ledBlinkTime = 200;
          blueState = "CONNECTED";
          CANDSENDCMD = 1;
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
        return;
      }else if (serBuf.val == "blueinit") {
        blueState = "INIT";
        return;
      }else if (serBuf.cmd == "show") {
        debugShow = !debugShow;        
        Serial.println("Debug is " + String(debugShow));
        return;
      }
      BTSerial.write(serBuf.origVal.c_str());
      BTSerial.write("\r\n");
    }
  }  

  for (int i = 0; i < 3; i++) {
    QueueCommand * cmd = queueCmds[i];
    String cmdStr = cmd->GetCommand();
    if (cmdStr != "" && CANDSENDCMD) {
      BTSerial.write(cmdStr.c_str());
      Serial.print(cmdStr);
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
      if (debugShow) 
      {
        for (int i = 0; i < 3; i++) {
          float ypri = ypr[i];
          if (yprMax[i] < ypri) yprMax[i] = ypri;
          if (yprMin[i] > ypri) yprMin[i] = ypri;
          float diff = abs(yprOld[i] - ypri);
          if (diff > 0.2) {
            yprOld[i] = ypri;
            Serial.println("for " + String(i)+" val="+String(ypri)+ " diff="+String(diff)+ " ("+String(yprMin[i])+","+String(yprMax[i])+")");
            //for 2 left right (-0.84,0.86)
            //1 low high (-0.84,0.86)
          }
        }
        //Serial.println(String(ypr[0]) + " " + String(ypr[1])+" " + String(ypr[2]));
      }
      yprActI();
   }
   //serprintln("int="+String(mpuInterrupt) + " fifoCount=" + String(fifoCount)+"/"+String(packetSize)+" i=" +String(input)+" o=" + String(output));
}


int driveLimit(int d) {
  if (abs(d) < 3) return 0;
  return d;
}
void yprActI() {
  int SCALE = 10;
  float yrScale = 0.86;
  int lrDiff = (int)(ypr[2]/yrScale*SCALE);
  int l = lrDiff;
  int r = -lrDiff;
  int frDiff = (int)(ypr[1]/yrScale*SCALE);
  if (frDiff > SCALE) {
    lCmd.AddCommand("l:0\n");
    rCmd.AddCommand("r:0\n");  
  }
  l -= frDiff;
  r -= frDiff;
  l = driveLimit(l);
  r = driveLimit(r);
  lCmd.AddCommand(String("l:")+String(l)+"\n");
  rCmd.AddCommand(String("r:")+String(r)+"\n");
}
