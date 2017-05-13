//Updated 18/4/17

// Import ESP8266 libraries
#include <ESP8266WiFi.h>
#include <WiFiUdp.h>

// WiFi parameters
const char* ssid = "ThomasWifi"; //Enter your WiFi network name here in the quotation marks
const char* password = "vanillamoon576"; //Enter your WiFi pasword here in the quotation marks

//Server details
unsigned int localPort = 5007;  //UDP send port
const char* ipAdd = "192.168.0.100"; //Server address
byte packetBuffer[512]; //buffer for incoming packets

const int devices=2; //Number of LED devices
const unsigned long devID[devices] = {31249122,23323231}; //Name of sensor
const unsigned long devType[devices] = {31,35}; //Type of sensor is button and RF server

const int devPin[devices] = {16,14}; //LED pin number 12 for LED2, 13 for LED1. 14 for RF and 16 for button.

//1 = 690 up then 310 down.  990 up, 370 down
//0== 245 up then 775 down.   310 up, 1050 down
const long t1=30;
const long t2=300;
const long t3=t2+1;
const long t4=700;
const byte msgLengths[4]={9,6,7,9};


long lastRead=millis();
byte rfBuffer=0;
byte bitBuffer=0;
long riseTime=lastRead;
long fallTime=lastRead;
long upTime=10;
long downTime=10;
bool fallFlag=false;
bool riseFlag=false;
bool newByte=false;
byte bitCount=0;
byte byteCount=0;
byte byteStore[9]={0,0,0,0,0,0,0,0,0};
bool record=false;
unsigned int msg=0;
long lastTriggered=0;
bool msgFlag=false;
byte msgType=0;
String outputString="";
long lastMsgTime=0;
String lastString1="";
String lastString2="";
boolean transmitFlag=false;
long lastTransmit=0;


//Button related
bool lastButtonState=0;
bool buttonState=0;
long buttonTriggerTime=millis();
long currentTime=millis();
bool primer[4]={0,0,0,0};

WiFiUDP Udp; //Instance to send packets


void setup() {
  //For Mosfets if connected
  pinMode(12, OUTPUT); //Set as output
  digitalWrite(12, 0); //Turn off LED while connecting
  pinMode(13, OUTPUT); //Set as output
  digitalWrite(13, 0); //Turn off LED while connecting

  //button setup
  pinMode(devPin[devices],INPUT_PULLUP);
  
  // put your setup code here, to run once:
  Serial.begin(115200);
  ConnectWifi();  //Only include connection if in produduction mode

  delay(2000); //Time clearance to ensure registration
  //pinMode(indicatorPin,OUTPUT);
  for (int i=0; i<devices;i++){
    SendUdpValue(0,devID[i],devType[i]); //Register LED on server
  }
  Serial.println("Ready");
}

void ConnectWifi() {
  // Connect to WiFi
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("");
  Serial.print("WiFi connected with IP: ");
  // Print the IP address
  Serial.println(WiFi.localIP());

  //Open the UDP monitoring port
  Udp.begin(localPort);
  Serial.print("Udp server started at port: ");
  Serial.println(localPort);
}

void loop() {
  if (WiFi.status() != WL_CONNECTED) { //Reconnect if required
    ConnectWifi();
  }
  //For detecting rising and falling edges of a waveform
  if (micros()-lastRead>10) { //Takes 56us to fill buffer
    rfBuffer=rfBuffer<<1;
    rfBuffer=rfBuffer+digitalRead(devPin[0]);
    if (rfBuffer==7) { //Indicates a rise 00001111
      riseTime=micros();
      downTime=riseTime-fallTime;
      riseFlag=true;
    }
    else if (rfBuffer==240) { //Indicates a fall 11110000
      fallTime=micros();
      upTime=fallTime-riseTime;
      fallFlag=true;
    }
  }
  //For catching bit durations and adding to the buffer, then saying when a new byte is ready
  if (riseFlag) {
    riseFlag=false;
    if (downTime>t4+t1) { //Message must have ended since no new bits detected
      bitBuffer=0;
      bitCount=0;
      byteCount=0;
      if (record) { //stop recording if bits are wrong
        record=false;
      }
    }
    else if (upTime>t1 && upTime<t2) { //Indicates a 0
      bitBuffer=bitBuffer<<1;
      bitCount=bitCount+1;
      if (bitCount>=8 && record) {
        newByte=true;
      }
    }
    else if (upTime>t3 && upTime<t4) { //Indicates a 1
      bitBuffer=bitBuffer<<1;
      bitBuffer=bitBuffer+1;
      bitCount=bitCount+1;
      if (bitCount>=8 && record) {
        newByte=true;
      }
    }
    if ((bitBuffer==120 || bitBuffer==121 || bitBuffer==122 || bitBuffer==123) && (!record)) {
      //Serial.println("Preamble met");
      newByte=true;
      record=true;
      msgType=bitBuffer-120;
    }
  }
  //For recognising if a preable criteria is met and starting record
  if (newByte) {
    newByte=false;
    //Serial.println(bitBuffer); //Uncomment to get a stream of read bytes----------
    if (record) {
      byteStore[byteCount]=bitBuffer;
      bitBuffer=0;
      bitCount=0;
      byteCount=byteCount+1;
      //Serial.println(byteCount);
      if (byteCount>=msgLengths[msgType]) {
        //Serial.println("-END-");
        msgFlag=true;
        lastMsgTime=micros();
        record=false;
        byteCount=0;
      }
    }
  }
  //Process messages
  if (msgFlag) {
    msgFlag=false;
    if (millis()<lastTriggered) {
      lastTriggered=millis();
    }
    
    //In transmission form
    unsigned long dID = 0;
    dID = byteStore[1] * 16777216 + byteStore[2] * 65536 + byteStore[3] * 256 + byteStore[4];
    outputString=String(byteStore[0]-120) + ',' + dID;
    if (msgType==0 || msgType==3) {
      unsigned long sMsg = 0;
      sMsg  = byteStore[5] * 16777216 + byteStore[6] * 65536 + byteStore[7] * 256 + byteStore[8];
      outputString=outputString +','+sMsg;
    }
    else if (msgType==1) {
      byte sMsg = 0;
      sMsg = byteStore[5];
      outputString=outputString +','+sMsg;
    }
    else if (msgType==2) {
      unsigned int sMsg = 0;
      sMsg  = byteStore[5] * 256 + byteStore[6];
      outputString=outputString +','+sMsg;
    }
    if ((outputString==lastString1 || outputString==lastString2) && (millis()-lastTransmit>550)) {
      lastTransmit=millis();
      SendUdpString(outputString);
      //Serial.println(outputString);
    }
    lastString2=lastString1;
    lastString1=outputString;
  }
}

void CheckButton() {
  buttonState=(!digitalRead(devPin[devices]));
  currentTime=millis();
  if (buttonState!=lastButtonState) {
    if (buttonState && currentTime-buttonTriggerTime>300) {
      SendUdpValue(1,devID[devices],1);
      buttonTriggerTime=currentTime;
      primer[0]=1;
      primer[1]=1;
      primer[2]=1;
      primer[3]=1;
    }
    else if (!buttonState) {
      primer[0]=0;
      primer[1]=0;
      primer[2]=0;
      primer[3]=0;
    }
  }
  lastButtonState=buttonState;
  if (primer[0] && (currentTime-buttonTriggerTime>600)) {
    SendUdpValue(1,devID[devices],2);
    primer[0]=0;
  }
  else if (primer[1] && (currentTime-buttonTriggerTime>1500)) {
    SendUdpValue(1,devID[devices],3);
    primer[1]=0;
  }
  else if (primer[2] && (currentTime-buttonTriggerTime>4000)) {
    SendUdpValue(1,devID[devices],4);
    primer[2]=0;
  }
  else if (primer[3] && (currentTime-buttonTriggerTime>8000)) {
    SendUdpValue(0,devID[devices],devType[devices]);  //Register
    primer[3]=0;
  }
}

void SendUdpString(String msg) {
  //Print GPIO state in //Serial
  Serial.print("-UDP sent: ");
  Serial.println(msg);
  // send a message, to the IP address and port
  Udp.beginPacket(ipAdd,localPort);
  Udp.print(msg);
  Udp.endPacket();
}

void SendUdpValue(byte type, unsigned long devID, unsigned long value) {
  //Print GPIO state in //Serial
  Serial.print("-Value sent via UDP: ");
  Serial.println(String(type) + "," + String(devID) + "," + String(value));

  // send a message, to the IP address and port
  Udp.beginPacket(ipAdd,localPort);
  Udp.print(String(type));
  Udp.write(",");
  Udp.print(String(devID));
  Udp.write(",");
  Udp.print(String(value)); //This is the value to be sent
  Udp.endPacket();
}
