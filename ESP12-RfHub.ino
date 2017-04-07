// Import ESP8266 libraries
#include <ESP8266WiFi.h>
#include <WiFiUdp.h>

const char* sensorID1 = "BUT008"; //Name of sensor
const char* sensorID2 = "BUT009"; //Name of sensor
const char* sensorID3 = "BUT010"; //Name of sensor
const char* deviceDescription1 = "RemoteButtonA";
const char* deviceDescription2 = "RemoteButtonB";
const char* deviceDescription3 = "DoorSensor";
const int readPin = 16;

//1 = 690 up then 310 down.  990 up, 370 down
//0== 245 up then 775 down.   310 up, 1050 down
const long t1=200;
const long t2=600;
const long t3=601;
const long t4=1700;


long lastRead=millis();
byte rfBuffer=0;
byte bitBuffer=0;
long riseTime=lastRead;
long fallTime=lastRead;
long upTime=10;
long downTime=10;
bool fallFlag=false;
bool newByte=false;
byte bitCount=0;
byte byteCount=0;
byte byteStore[8]={0,0,0,0,0,0,0,0};
bool record=false;
unsigned int msg=0;
long lastTriggered=0;
bool msgFlag=false;



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
  ConnectWifi();

  delay(2000); //Time clearance to ensure registration
  //pinMode(indicatorPin,OUTPUT);
  SendUdpValue("REG",sensorID1,String(deviceDescription1)); //Register LED on server
  SendUdpValue("REG",sensorID2,String(deviceDescription2)); //Register LED on server
  SendUdpValue("REG",sensorID3,String(deviceDescription3)); //Register LED on server


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
  if (micros()-lastRead>20) { //Takes 160us to fill buffer
    rfBuffer=rfBuffer<<1;
    rfBuffer=rfBuffer+digitalRead(readPin);
    if (rfBuffer==7) { //Indicates a rise 00001111
      riseTime=micros();
      downTime=riseTime-fallTime;
    }
    else if (rfBuffer==240) { //Indicates a fall 11110000
      fallTime=micros();
      upTime=fallTime-riseTime;
      fallFlag=true;
    }
  }
  //For catching bit durations and adding to the buffer, then saying when a new byte is ready
  if (fallFlag) {
    fallFlag=false;
    if (downTime>t4+t1) { //Message must have ended since no new bits detected
      bitBuffer=0;
      bitCount=0;
      byteCount=0;
    }
    else if (upTime>t1 && upTime<t2) { //Indicates a 0
      bitBuffer=bitBuffer<<1;
      bitCount=bitCount+1;
      if (bitCount>=8) {
        bitCount=0;
        newByte=true;
      }
    }
    else if (upTime>t3 && upTime<t4) { //Indicates a 1
      bitBuffer=bitBuffer<<1;
      bitBuffer=bitBuffer+1;
      bitCount=bitCount+1;
      if (bitCount>=8) {
        bitCount=0;
        newByte=true;
      }
    }
  }
  //For recognising if a preable criteria is met and starting record
  if (newByte) {
    newByte=false;
    //Serial.println(bitBuffer); //Uncomment to get a stream of read bytes----------
    if (record) {
      byteStore[byteCount]=bitBuffer;
      byteCount=byteCount+1;
      if (byteCount>=2) {
        msg=byteStore[0]*256+byteStore[1];
        msgFlag=true;
        //Serial.println(msg);  //Uncomment to see each integer message------
        record=false;
      }
    }
    else if (bitBuffer==217 || bitBuffer==225) { //Preamble condition met. 225 is preamble for Door sensor
      record=true;
      //Serial.println("Preamble met");
    }
  }
  //Process messages
  if (msgFlag) {
    msgFlag=false;
    if (millis()<lastTriggered) {
      lastTriggered=millis();
    }
    if ((msg==22882) && millis()-lastTriggered>300) {
      lastTriggered=millis();
      //Serial.println("Button A triggered");
      SendUdpValue("LOG",sensorID1,"press");
    }
    if (msg==22884 && millis()-lastTriggered>300) {
      lastTriggered=millis();
      //Serial.println("Button B triggered");
      SendUdpValue("LOG",sensorID2,"press");
    }
    if (msg==38726 && millis()-lastTriggered>10000) {
      lastTriggered=millis();
      //Serial.println("Door triggered");
      SendUdpValue("LOG",sensorID3,"press");
    }
  }
}

void SendUdpValue(String type, String sensorID, String value) {
  //Print GPIO state in serial
  Serial.print("-Value sent via UDP: ");
  Serial.println(type + "," + sensorID + "," + value);

  // send a message, to the IP address and port
  Udp.beginPacket(ipAdd,localPort);
  Udp.print(type);
  Udp.write(",");
  Udp.print(sensorID);
  Udp.write(",");
  Udp.print(value); //This is the value to be sent
  Udp.endPacket();
}
