//Updated 16/4/17

// Import ESP8266 libraries
#include <ESP8266WiFi.h>
#include <WiFiUdp.h>

const unsigned long devID[3] = {24249122,12312414,12313}; //Name of sensor
const unsigned long devType[3] = {5,13,8}; //Type of sensor is temperature

const int readPin = 16; //16 for v0.1 of PWM board print

//1 = 690 up then 310 down.  990 up, 370 down
//0== 245 up then 775 down.   310 up, 1050 down
const long t1=200;
const long t2=600;
const long t3=601;
const long t4=1200;
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
byte byteStore[8]={0,0,0,0,0,0,0,0};
bool record=false;
unsigned int msg=0;
long lastTriggered=0;
bool msgFlag=false;
byte msgType=0;
String outputString="";



// WiFi parameters
const char* ssid = "ThomasWifi"; //Enter your WiFi network name here in the quotation marks
const char* password = "vanillamoon576"; //Enter your WiFi pasword here in the quotation marks

//Server details
unsigned int localPort = 5007;  //UDP send port
const char* ipAdd = "192.168.0.100"; //Server address
byte packetBuffer[512]; //buffer for incoming packets

WiFiUDP Udp; //Instance to send packets


void setup() {
  //For Mosfets if connected
  pinMode(12, OUTPUT); //Set as output
  digitalWrite(12, 0); //Turn off LED while connecting
  pinMode(13, OUTPUT); //Set as output
  digitalWrite(13, 0); //Turn off LED while connecting
  
  // put your setup code here, to run once:
  Serial.begin(115200);
  //ConnectWifi();  //Only include connection if in produduction mode

  delay(2000); //Time clearance to ensure registration
  //pinMode(indicatorPin,OUTPUT);
  SendUdpValue(0,devID[0],devID[0]); //Register LED on server
  SendUdpValue(0,devID[1],devID[1]); //Register LED on server
  SendUdpValue(0,devID[2],devID[2]); //Register LED on server


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
  //For detecting rising and falling edges of a waveform
  if (micros()-lastRead>10) { //Takes 80us to fill buffer
    rfBuffer=rfBuffer<<1;
    rfBuffer=rfBuffer+digitalRead(readPin);
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
      //Serial.print("upTime,");     //Uncomment to get a read-out of rf pulse lengths
      //Serial.println(upTime);        //Uncomment to get a read-out of rf pulse lengths
      //Serial.print("downTime,");   //Uncomment to get a read-out of rf pulse lengths
      //Serial.println(downTime);      //Uncomment to get a read-out of rf pulse lengths
      if (bitCount>=8 && record) {
        newByte=true;
      }
    }
    else if (upTime>t3 && upTime<t4) { //Indicates a 1
      bitBuffer=bitBuffer<<1;
      bitBuffer=bitBuffer+1;
      bitCount=bitCount+1;
      //Serial.print("upTime,");     //Uncomment to get a read-out of rf pulse lengths
      //Serial.println(upTime);        //Uncomment to get a read-out of rf pulse lengths
      //Serial.print("downTime,");   //Uncomment to get a read-out of rf pulse lengths
      //Serial.println(downTime);      //Uncomment to get a read-out of rf pulse lengths
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
    //else if (bitCount>127) {
    //  bitCount=0;
    //}
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
    outputString="";
    for (int i=0; i<msgLengths[msgType]; i++) {
      outputString=outputString+byteStore[i]+',';
    }
    Serial.println(outputString);
    
//    if ((msg==22882) && millis()-lastTriggered>300) {
//      lastTriggered=millis();
//      Serial.println("Button A triggered");
//      SendUdpValue(1,devID[0],0);
//    }
//    if (msg==22884 && millis()-lastTriggered>300) {
//      lastTriggered=millis();
//      Serial.println("Button B triggered");
//      SendUdpValue(1,devID[1],0);
//    }
//    if (msg==43904 && millis()-lastTriggered>300) {
//      lastTriggered=millis();
//      Serial.println("Remote 2 triggered");
//      SendUdpValue(1,devID[1],0);
//    }
//    if (msg==38726 && millis()-lastTriggered>10000) {
//      lastTriggered=millis();
//      Serial.println("Door triggered");
//      SendUdpValue(1,devID[2],0);
//    }
  }
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
