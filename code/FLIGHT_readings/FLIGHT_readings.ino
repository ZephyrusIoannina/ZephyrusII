//Include the required libraries

#include <Wire.h>
#include <SPI.h>
#include <Timer.h>
#include <SD.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h> //Pressure,Temp,Humidity
#include <Adafruit_GPS.h> // Gps
#include <RH_RF95.h> // Radio lora 433Mhz
#include <Adafruit_BNO055.h>// Absolute orientation sensor
#include <utility/imumaths.h>// Absolute orientation maths
#include <PID_v1.h> // Pid Code
#include <Servo.h> // Servo library

// Blinky on receipt
#define LED 13

//sd card set chipselect
const int chipSelect = 11;

char serialdata[14]; //  This is the buffer from the serial port
char initcom[4]; // Check the first 3 letters of each command recieved - Look for command
char noletters[10];// The other 9 letters are command values

//define timers ---------------------------------
Timer t;
Timer t2;
Timer ta;
Timer tb;
Timer tc;
//PID parameters------------------------------------
// Define Variables we'll be connecting to
double Setpoint, Input, Output;

// Define the aggressive and conservative Tuning Parameters
double aggKp = 4, aggKi = 0.2, aggKd = 1;
double consKp = 1, consKi = 0.05, consKd = 0.25;

//Specify the links and initial tuning parameters
PID myPID(&Input, &Output, &Setpoint, consKp, consKi, consKd, DIRECT);

float PIDstatus;  // This value is to be set with a command to choose the servo mode
//float testDiff; // Testing

//Battery levels Parameters -------------------------------------
#define VBATPIN A7

//servo parameters
Servo myservoL;  // Create  servo object to control servo Left
Servo myservoR;  // Create  servo object to control servo Right

int posR = 115;    // Variable to store the servo position Right
int posL = 5;    // Variable to store the servo position Left
int maxAngle; // Max allowed servo angle  -  Max allowed is 110 degrees
float maxAng; // Same as the above but in float 

//BNO055 Parameters-------------------------------------------------
Adafruit_BNO055 bno = Adafruit_BNO055(55);

//BME280 parameters ----------------------------------------------------
#define SEALEVELPRESSURE_HPA (1013.25)
Adafruit_BME280 bme; // I2C
int P, T, H, Ap, Apold; // Integers to be send -  pressure,temp,humidity,Altitude(by pressure), previous altitude

//Radio Parameters -------------------------
/* for feather m0  */

#define RFM95_CS 8
#define RFM95_RST 4
#define RFM95_INT 3

// Change to 434.0 or other frequency, must match RX's freq!
#define RF95_FREQ 433.55


// Singleton instance of the radio driver
RH_RF95 rf95(RFM95_CS, RFM95_INT);


char apostoli[100];
char apostoliB[100];
char fdata[100]; // Selected flight parameters data
char reply[10];

//GPS Parameters -------------------------
// Set GPSECHO to 'false' to turn off echoing the GPS data to the Serial console
// Set to 'true' if you want to debug and listen to the raw GPS sentences
#define GPSECHO false
// this keeps track of whether we're using the interrupt
// off by default!
boolean usingInterrupt = false;
//GPS hardware serial - Cannot use software serial
Adafruit_GPS GPS(&Serial1);



//--------------------------------------------------|
//                    WAYPOINTS                     |
//--------------------------------------------------|
//Please enter the latitude and longitude of your   |
// A site to acquire cordinates
//https://www.gps-coordinates.net/
//desired destination:  (Test - School)         39.660363,20.851830,  |
//39.698092, 20.772974
//north - 39.719097, 20.820682
//north + 39.722688, 20.835791
#define GEO_LAT                38.046410
#define GEO_LON                23.401782
//default ligkiades
//--------------------------------------------------|
// Navigation location
float targetLat = GEO_LAT; // Latitude of target
float targetLon = GEO_LON; // longitude of target
float targetAlt = 100; //  Threshold altitude after which we are  in landing mode 
float startingAlt = 500; // Altitude when Pid starts
// Trip distance
float tripDistance;  // Distance from target
long dist;            // Distance from target in long for beaming to the groundstation
float fHeading;    // direction to the set target in relation to the north
int heading;      //  direction to the set target in relation to the northh in Int for beaming to the groundstation
float maxaccX;
float maxaccY;
float maxaccZ;
//float maxACCEL; // Max acceleration in each calculation cycle
//double headingpid;
boolean isStarted = false;
long interval;
long previousMillis;


//Reply Messages -------------------------
char message1[] = {"Flight_Reply1"};
char message2[] = {"Flight_Reply2"};
char message3[] = {"target_UOI"};
char message4[] = {"Dump_SDserial"};
char message5[] = {"Erase_SD_data"};

int val;

void setup()
{
  //Initialize serial connection for debugging
  Serial.begin(9600);

  // GPS SETUP----------------------------------------
  GPS.begin(9600);
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
  // GPS.sendCommand(PMTK_SET_NMEA_UPDATE_200_MILLIHERTZ);
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ); // Set the update rate
  GPS.LOCUS_StartLogger(); //start GPS internal memory logger
  /*  GPS.sendCommand("$PMTK185,1*23");  // Stops the logger.
  */


  //Servo attach to pins
  myservoR.attach(5);  // attaches the servo on pin 5 to the servo object
  myservoL.attach(10);  // attaches the servo on pin 10 to the servo object

  myservoR.write(posR);
  myservoL.write(posL);

  PIDstatus = 23;
  maxAngle = 60;
  //  testDiff = -180;
  //PIDstatus

  delay(2000);

  // BME280 SETUP ---------------------------------------
  bool status;
  status = bme.begin();
  if (!status) {
    Serial.println("Could not find a valid BME280 sensor, check wiring!");
    //while (1);
  }
  // BNO055 SETUP --------------------------------
  if (!bno.begin())
  {
    /* There was a problem detecting the BNO055 ... check your connections */
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    //while(1);
  }
  delay(1000);
  bno.setExtCrystalUse(true);

  maxaccX = 0;
  maxaccY = 0;
  maxaccZ = 0;

  //Radio SETUP --------------------------------

  pinMode(RFM95_RST, OUTPUT);
  digitalWrite(RFM95_RST, HIGH);
  delay(100);

  Serial.println("Feather LoRa TX Test!");

  // manual reset
  digitalWrite(RFM95_RST, LOW);
  delay(10);
  digitalWrite(RFM95_RST, HIGH);
  delay(10);

  while (!rf95.init()) {
    Serial.println("LoRa radio init failed");
    //while (1);
  }
  Serial.println("LoRa transmit radio init OK!");
  // Defaults after init are 434.0MHz, modulation GFSK_Rb250Fd250, +13dbM
  if (!rf95.setFrequency(RF95_FREQ)) {
    Serial.println("setFrequency failed");
    //while (1);
  }
  Serial.print("Set Freq to: "); Serial.println(RF95_FREQ);

  // Defaults after init are 434.0MHz, 13dBm, Bw = 125 kHz, Cr = 4/5, Sf = 128chips/symbol, CRC on
  // The default transmitter power is 13dBm, using PA_BOOST.
  // If you are using RFM95/96/97/98 modules which uses the PA_BOOST transmitter pin, then
  // you can set transmitter powers from 5 to 23 dBm:
  rf95.setTxPower(23, false);

  delay(500); //for everything to initialize

  //SD Card SETUP ---------------

  // put after Radio initiallization
  //8 is the actual CS pin for the feather.
  //It is inaccesible and used by the radio but must be HIGH in order for sd to work no matter what CS pin for sd is used.
  // pinMode(8, OUTPUT);//unrem if radio off
  //digitalWrite(8, HIGH);//unrem if radio off
  Serial.print("Initializing SD card...");
  if (!SD.begin(chipSelect)) {
    Serial.println("initialization failed!");
    //return;
  }
  Serial.println("initialization done.");

  //PID setup----------------------------------
  // The input is the value for headingpid
  Setpoint = 0; // Is made the value of the actual heading
  //turn the PID on
  myPID.SetMode(AUTOMATIC);

  //Timers SETUP ---------------
  t.every (1000, steiledata); // After how much time is the function senddata  
  t2.every(5000, steiledataB); //After how much time is the function senddata
  tb.every(200, BNOvalues); //After how much time is the function BNOvalues
  tc.every(1000, pidfunction);//After how much time is the function Pidfunction
}
//^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^ VOID LOOP ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
void loop()
{

  //Receive NMEA phrase from GPS -  Cannot be made into function---It doesnt work---
  char c = GPS.read();
  if (GPS.newNMEAreceived()) {
    // a tricky thing here is if we print the NMEA sentence, or data
    // we end up not listening and catching other sentences!
    // so be very wary if using OUTPUT_ALLDATA and trytng to print out data
    //Serial.println(GPS.lastNMEA());   // this also sets the newNMEAreceived() flag to false
  }
  if (!GPS.parse(GPS.lastNMEA())) {  // this also sets the newNMEAreceived() flag to false
    //return;  // we can fail to parse a sentence in which case we should just wait for another
    Serial.print("");
  }

  //update timers ------------------------
  t.update();//update timer
  t2.update(); //update timer
  tb.update();//update timer
  tc.update();//update timer


  // -------an pirame kapoia entoli tote... ------------

  uint8_t buf[RH_RF95_MAX_MESSAGE_LEN];
  uint8_t len = sizeof(buf);

  if (rf95.recv(buf, &len))
  {
    Serial.print("RSSI,");
    Serial.print(rf95.lastRssi(), DEC); Serial.print(",");
    Serial.println((char*)buf);

    // Everything received from the radio module is also saved on the sd card. In this  way we backup each modules data to both modules 
    /* File dataFile = SD.open("datalog.txt", FILE_WRITE);
    if (dataFile) {
      dataFile.println((char*)buf);
      dataFile.close();
    }*/

    //Split the buffer in pieces to check for commands
    memcpy(initcom, &buf[0], 3 ); // First three letters of what we received
    memcpy(noletters, &buf[3], 10 );// The following 9
    initcom[4] = '\0';  //End of array
    noletters[10] = '\0';  //End of array

    // Serial.println(initcom);
    // Serial.println(noletters);

  }
  if (strlen((char*)buf) > 0)
  {
    if (strcmp_P((char*)buf, "gpsA") == 0)    // Compares Ram to Flash memory
    {
      gpsA();
    }
    else if (strcmp_P((char*)buf, "gpsB") == 0)
    {
      gpsB();
    }
    else if (strcmp_P((char*)buf, "gpsC") == 0)
    {
      gpsC();
    }
    else if (strcmp_P((char*)buf, "gpsD") == 0)
    {
      gpsD();
    }
    else if (strcmp_P((char*)buf, "FdumpSD") == 0)
    {
      FdumpSD();
    }
    else if (strcmp_P((char*)buf, "FdeleteSD") == 0)
    {
      FdeleteSD();
    }
    else if (strcmp_P((char*)buf, "flight") == 0)
    {
      flightdata();
    }
    else if (strcmp_P((char*)initcom, "ang") == 0)
    {
      chAngle();
    }
    else if (strcmp_P((char*)initcom, "lat") == 0)
    {
      changelat();
    }
    else if (strcmp_P((char*)initcom, "lon") == 0)
    {
      changelon();
    }
    else if (strcmp_P((char*)initcom, "alt") == 0)
    {
      changealt();
    }
    else if (strcmp_P((char*)initcom, "pid") == 0)
    {
      changepid();
    }
    /*else if (strcmp_P((char*)initcom, "dif") == 0)
      {
      testDiff = atof(noletters);
      }*/

  }
  void clearRxBuf(); //καθαρισμός buffer λήψης
  memset(buf, 0, sizeof(buf)); //Deletes the variable buf


  //-----------------receive command from serial ----------------------

  while (Serial.available()) {    //As long as something is received in serial port
    Serial.readBytes(serialdata, 14); //We use readbytes as it used below for beaming to the groundstation,  Serial.readString is not converted
    Serial.println(serialdata);

    if (strcmp_P((char*)serialdata, "FdumpSD") == 0)
    {
      FdumpSD();
    }
    if (strcmp_P((char*)serialdata, "FdeleteSD") == 0)
    {
      FdeleteSD();
    }
    if (strcmp_P((char*)serialdata, "gpsC") == 0) {
      gpsC();
    }
    if (strcmp_P((char*)serialdata, "gpsA") == 0) {
      gpsA();
    }
    if (strcmp_P((char*)serialdata, "gpsB") == 0) {
      gpsB();
    }
    if (strcmp_P((char*)serialdata, "deleteGPS") == 0) {
      deleteGPS();
    }
    if (strcmp_P((char*)serialdata, "dumpGPS") == 0) {
      dumpGPS();
    }
    else {
    }
  }
  memset(serialdata, 0, sizeof(serialdata)); //Deletes the serialdata variable
  // if you want to debug, this is a good time to do it!
  if (GPSECHO)
    if (c) Serial.print(c);
}




