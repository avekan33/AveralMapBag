//MapBag code using the Adafruit Ultimate GPS module and the HMC6352 compass module (for tilt compensation, purchase the HMC6343 or any other such compass-
//while Honeywell has stopped manufacture of this module, getting your hands on one would improve your MapBag experience hundred-fold.

//Honywell HMC6352 Compass module from SparkFun Electronics
//https://www.sparkfun.com/products/7915
//Connect SDA to Analog Pin 4 (A4) and SCL to Analog Pin 5 (A5)
// Adafruit Ultimate GPS module
// using MTK3339 chipset
//    ------> http://www.adafruit.com/products/746

int dest = 1; //choose a location from the ten at the bottom of the code, with counting starting from zero of course!
//There is potential for more locations to be added.
#include <Adafruit_GPS.h>
//GPS library. Download Here:
//http://learn.adafruit.com/adafruit-ultimate-gps/arduino-wiring
#include <SoftwareSerial.h>
#include <Wire.h>
const int ne = 10;
const int e = 11;
const int se = 12;
const int sw = A0;
const int w = A1;
const int nw = A2;
const int n = A3;
int HMC6352Address = 0x42;
//To be honest, bascially all of the Wire library is impossible for me to understand. Use this guy's code, it does most of the work for you:
//http://recombine.net/blog/article/49
int slaveAddress;
byte headingData[2];
int i, headingValue;
//A lot of the comments here are from the example code I used to set up the GPS. From now on, if the comment doesn't have "-Averal" at the end, it's not me.
// If you're using a GPS module:
// Connect the GPS Power pin to 5V
// Connect the GPS Ground pin to ground
// If using software serial (sketch example default):
//   Connect the GPS TX (transmit) pin to Digital 3
//   Connect the GPS RX (receive) pin to Digital 2
// If using hardware serial (e.g. Arduino Mega):
//   Connect the GPS TX (transmit) pin to Arduino RX1, RX2 or RX3
//   Connect the GPS RX (receive) pin to matching TX1, TX2 or TX3

// If you're using the Adafruit GPS shield, change 
// SoftwareSerial mySerial(3, 2); -> SoftwareSerial mySerial(8, 7);
// and make sure the switch is set to SoftSerial

// If using software serial, keep these lines enabled
// (you can change the pin numbers to match your wiring):
SoftwareSerial mySerial(13, 2);
//Use any pin but 1/tx and 0/rx - these are for Serial communication with the computer
//If used, they will corrupt your data -Averal
Adafruit_GPS GPS(&mySerial);
// If using hardware serial (e.g. Arduino Mega), comment
// out the above six lines and enable this line instead:
//Not Recommended, just use SoftwareSerial, it works fine with the Mega -Averal
//Adafruit_GPS GPS(&Serial1);


// Set GPSECHO to 'false' to turn off echoing the GPS data to the Serial console
// Set to 'true' if you want to debug and listen to the raw GPS sentences. 
//GPSECHO prints random GPS garbage to the Serial Monitor. Useful to check if the GPS is working but counter-productive otherwise -Averal
#define GPSECHO  false

// this keeps track of whether we're using the interrupt
// off by default!
boolean usingInterrupt = false;
void useInterrupt(boolean); // Func prototype keeps Arduino 0023 happy

void setup()  
{
  slaveAddress = HMC6352Address >> 1;
  Wire.begin();
  
  pinMode(n, OUTPUT);
  pinMode(nw, OUTPUT);
  pinMode(w, OUTPUT);
  pinMode(sw, OUTPUT);
  pinMode(se, OUTPUT);
  pinMode(e, OUTPUT);
  pinMode(ne, OUTPUT);  
  // connect at 115200 so we can read the GPS fast enough and echo without dropping chars
  // also spit it out
  Serial.begin(115200);
  //Make sure when you're looking at the Serial Monitor that it's set to 115200 baud or you'll see garbage -Averal
  Serial.println("Adafruit GPS library basic test!");

  // 9600 NMEA is the default baud rate for Adafruit MTK GPS's- some use 4800
  GPS.begin(9600);
  
  // uncomment this line to turn on RMC (recommended minimum) and GGA (fix data) including altitude
  //GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
  // uncomment this line to turn on only the "minimum recommended" data
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCONLY);
  //Leave it like this or your GPS will very often get data wrong. - Averal
  // For parsing data, we don't suggest using anything but either RMC only or RMC+GGA since
  // the parser doesn't care about other sentences at this time
  
  // Set the update rate
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);   // 1 Hz update rate
  // For the parsing code to work nicely and have time to sort thru the data, and
  // print it out we don't suggest using anything higher than 1 Hz

  // Request updates on antenna status, comment out to keep quiet
  GPS.sendCommand(PGCMD_ANTENNA);

  // the nice thing about this code is you can have a timer0 interrupt go off
  // every 1 millisecond, and read data from the GPS for you. that makes the
  // loop code a heck of a lot easier!
  useInterrupt(true);

  delay(1000);
  // Ask for firmware version
  mySerial.println(PMTK_Q_RELEASE);
}


// Interrupt is called once a millisecond, looks for any new GPS data, and stores it
SIGNAL(TIMER0_COMPA_vect) {
  char c = GPS.read();
  // if you want to debug, this is a good time to do it!
#ifdef UDR0
  if (GPSECHO)
    if (c) UDR0 = c;  
    // writing direct to UDR0 is much much faster than Serial.print 
    // but only one character can be written at a time. 
#endif
}

void useInterrupt(boolean v) {
  if (v) {
    // Timer0 is already used for millis() - we'll just interrupt somewhere
    // in the middle and call the "Compare A" function above
    OCR0A = 0xAF;
    TIMSK0 |= _BV(OCIE0A);
    usingInterrupt = true;
  } else {
    // do not call the interrupt function COMPA anymore
    TIMSK0 &= ~_BV(OCIE0A);
    usingInterrupt = false;
  }
}

uint32_t timer = millis();
void loop()                     // run over and over again
{
  // in case you are not using the interrupt above, you'll
  // need to 'hand query' the GPS, not suggested :(
  if (! usingInterrupt) {
    // read data from the GPS in the 'main loop'
    char c = GPS.read();
    // if you want to debug, this is a good time to do it!
    if (GPSECHO)
      if (c) Serial.print(c);
  }
  // This code shows how to listen to the GPS module in an interrupt
 // which allows the program to have more 'freedom' - just parse
 // when a new NMEA sentence is available! Then access data when
 // desired.
  // if a sentence is received, we can check the checksum, parse it...
  if (GPS.newNMEAreceived()) {
    // a tricky thing here is if we print the NMEA sentence, or data
    // we end up not listening and catching other sentences! 
    // so be very wary if using OUTPUT_ALLDATA and trytng to print out data
    //Serial.println(GPS.lastNMEA());   // this also sets the newNMEAreceived() flag to false
  
    if (!GPS.parse(GPS.lastNMEA()))   // this also sets the newNMEAreceived() flag to false
      return;  // we can fail to parse a sentence in which case we should just wait for another
  }

  // if millis() or timer wraps around, we'll just reset it
  if (timer > millis())  timer = millis();

  // approximately every 2 seconds or so, print out the current stats
  if (millis() - timer > 2000) { 
    timer = millis(); // reset the timer
    
    Serial.print("\nTime: ");
    Serial.print(GPS.hour, DEC); Serial.print(':');
    Serial.print(GPS.minute, DEC); Serial.print(':');
    Serial.print(GPS.seconds, DEC); Serial.print('.');
    Serial.println(GPS.milliseconds);
    Serial.print("Date: ");
    Serial.print(GPS.day, DEC); Serial.print('/');
    Serial.print(GPS.month, DEC); Serial.print("/20");
    Serial.println(GPS.year, DEC);
    Serial.print("Fix: "); Serial.print((int)GPS.fix);
    Serial.print(" quality: "); Serial.println((int)GPS.fixquality); 
    if (GPS.fix) {
    float myLat = DEGM_to_DDEG(GPS.latitude);
    if(GPS.lat == 'S'){//If the latitude coordinate is in the southern hemisphere, it should be set to a negative value -Averal
      myLat = -myLat;
    }
    Serial.print("myLat: ");
    Serial.println(myLat);
    float myLong = DEGM_to_DDEG(GPS.longitude);
    if(GPS.lon == 'W'){//If the longitude coordinate is in the western hemisphere, it should also be set to a negative value -Averal
      myLong = -myLong;
    }
    Serial.print("myLong: ");
    Serial.println(myLong);
    float goLat = destlat(dest); //destination latitude coordinate -Averal
    Serial.print("goLat: ");
    Serial.println(goLat);
    float goLong = destlong(dest);//destination longitude coordinate -Averal
    Serial.print("goLong: ");
    Serial.println(goLong);
    float dy = goLat - myLat;
    float dx =goLong - myLong;
    Serial.println(dy);
    Serial.println(dx);
    
    float goAngle = calculateDirection(myLat, myLong, goLat, goLong); //finds the angle to the destination -Averal
    Wire.beginTransmission(slaveAddress);
    Wire.write("A");
    Wire.endTransmission(); //If your code is getting hung up here, check your wiring and soldering -Averal
    delay(10);
    Wire.requestFrom(slaveAddress, 2);
    i = 0;
    
    while(Wire.available() && i<2){
      headingData[i] = Wire.read();
      i++;
    }
    headingValue = headingData[0]*256 + headingData[1];
    float heading = (headingValue/10)+((headingValue%10)/100); //This truncates the value, but it was never that accurate in the first place, so it doesn't matter -Averal
    delay(500);
    
    
    Serial.print("goAngle: ");
    Serial.println(goAngle);
    if(goAngle>heading){ //My own ingenious code! -Averal
      heading = -heading; //If the angle to the destination is greater than our own heading, just find the difference -Averal
    }
    else{
      heading = -(heading - goAngle); //If it's the other way around, subtract the difference from 360 -Averal
      goAngle = 360;
    }
    float letsgo = goAngle + heading; //Add the two for our final turning angle -Averal
    
    
    
    if(letsgo>337.5 || letsgo<=22.5){
      pulse(n);
      Serial.println("Pulsing North..."); }//north
    else if(letsgo>22.5 && letsgo<=75){
      pulse(ne);
      Serial.println("Pulsing Northeast...");}//northeast
    else if(letsgo>75 && letsgo<=127.5){
      pulse(e);
      Serial.println("Pulsing East...");} //east
    else if(letsgo>127.5 && letsgo<=180){
      pulse(se);
      Serial.println("Pulsing Southeast...");} //southeast
    else if(letsgo>180 && letsgo<=232.5){
      pulse(sw);
      Serial.println("Pulsing Southwest...");}//southwest
    else if(letsgo>232.5 && letsgo<=285){
      pulse(w);
      Serial.println("Pulsing West...");}//west
    else
   {
      pulse(nw);
      Serial.println("Pulsing Northwest...by default or otherwise...");//northwest, I guess it'll also pulse northwest by default...so yeah -Averal
   }
    }else{
       Serial.println("OHHHHH NOOOOOO");//This might become kind of annoying after a while -Averal
    } 
    //If you're wondering why there's no south, it's because I initially ran out of pins...It's a long story. You can put it in if you want -Averal
  }
}

float destlat (int i){ //latitude grabbing function -Averal
  float table[10]={
    // latitude coordinate in decimal degrees
    //only accurate to 4 digits after the decimal point
     37.7645, //BSE (Mission Dolores Academy)
    37.7141, //My house...please don't stalk me -Averal
    37.7786, //S.F. City Hall
    37.7311, //Lowell H.S.
    41.8837, // Chicago City Hall
    37.7753, //SFO Int'l Airport
    37.3681, //Papalote Mexican Grill -AWESOME place to get a burrito if you're in S.F. -Averal
    37.8112, //Grandparents' house
    37.7797 //Sutro Baths
  };
  return table[i];}
  
float destlong(int i){
  float table[10]={
    -122.4280, //same order of locations as in destlat()
    -122.4696,
    -122.4182,
    -122.4836,
    -87.6324,
    -122.4193,
    -122.4210,
    -122.1062,
    -122.4778,
    -122.5138};
    return table[i];}
    
 float DEGM_to_DDEG(float b){
   int a  =b/100; //credit to Nathan from S.F. -Averal
   b = b-(a*100);
   b = b/60;
   b = b + a;
   return b;}
   
 void pulse(const int x){ 
   digitalWrite(x, HIGH); //Easy pulsing function -Averal
   delay(1000);
   digitalWrite(x, LOW);
   delay(1000);}
   
float calculateDirection (float lat1, float long1, float lat2, float long2) { //I got this function from Mattori NYC 2012's code. That guy won't respond to my emails, 
//so I'm not sure he would respond to yours -Averal
  Serial.println("Begin of calculateDirection");
  // returns initial course in degrees (North=0, West=270) from
  // position 1 to position 2, both specified as signed decimal-degrees
  // latitude and longitude.
  float dlon = radians(long2-long1);
  lat1 = radians(lat1);
  lat2 = radians(lat2);
  float a1 = sin(dlon) * cos(lat2);
  float a2 = sin(lat1) * cos(lat2) * cos(dlon);
  a2 = cos(lat1) * sin(lat2) - a2;
  a2 = atan2(a1, a2);
  if (a2 < 0.0) {
    a2 += TWO_PI;
  }
  return degrees(a2);
}
void cpulse(int i){
  digitalWrite(i, HIGH);
}
void pulseAll(){//Constant pulsing function to indicate a GPS error -Averal
  cpulse(n);
  cpulse(ne);
  cpulse(e);
  cpulse(se);
  cpulse(sw);
  cpulse(w);
  cpulse(nw);
} 

float GPSangle(){ //I'm pretty sure this function doesn't work, but I'll leave it there for documentation's sake -Averal
  float x = DEGM_to_DDEG(GPS.longitude);
  float y = DEGM_to_DDEG(GPS.latitude);
  delay(30000);
  float w = DEGM_to_DDEG(GPS.longitude);
  float z = DEGM_to_DDEG(GPS.latitude);
  return calculateDirection(y, x, z, w);  
}
