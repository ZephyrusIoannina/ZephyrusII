//Include the required libraries

#include <Wire.h>
#include <SPI.h>
#include <Timer.h> 
#include <SD.h> 
#include <Adafruit_Sensor.h>
#include "SparkFunISL29125.h" //uv sensor
#include "Adafruit_CCS811.h" //vocs sensor
#include <RH_RF95.h> //radio lora 433Mhz
#include "Adafruit_VEML6070.h" //UV intensity sensor


// Blinky on receipt
#define LED 13

//sd card set chipselect
const int chipSelect = 10;

//---Variables for synchronisation
char trig[2];// First letter buffer
unsigned long beforeMillis;
unsigned long afterMillis;
unsigned long duration;

//Battery levels Parameters -----------------------
#define VBATPIN A7

// Commands for comparison to flash memory
/*const char s1[] PROGMEM = {"epayload1"};
const char s2[] PROGMEM = {"epayload2"};
const char s3[] PROGMEM = {"epayload3"};
const char s4[] PROGMEM = {"epayload4"};
const char s5[] PROGMEM = {"epayload5"};
*/
//define timers -------------------
Timer t;

//UV sensor Parameters -------------------------
Adafruit_VEML6070 uv = Adafruit_VEML6070();

//RGB sensor parameters ------------------------
SFE_ISL29125 RGB_sensor;// Declare sensor object

//VOCS sensor parameters ------------------------
Adafruit_CCS811 ccs;
float temp;
//INTEGERS to be SEND ---------------------------------

int UV,VOCS,CO2; //integers to be send -  UV intensity,vocs(ppb), co2(ppm),RED, GREEN, BLUE 


//Radio Parameters -------------------------
/* for feather m0  */
#define RFM95_CS 8
#define RFM95_RST 4
#define RFM95_INT 3

// Change to 434.0 or other frequency, must match RX's freq!
#define RF95_FREQ 433.5
// Singleton instance of the radio driver
RH_RF95 rf95(RFM95_CS, RFM95_INT);

char* edafos;
char serialdata[14]; //  This is the buffer of the serial port
char apostoliC[80];

char message1[]={"Payload_Reply1"};
char message2[]={"Payload_Reply2"};
char message3[]={""};
char message4[]={"Dump_SD_serial"};
char message5[]={"Erase__SD_data"};

void setup()
{
  
  //Initialize serial connection for debugging
  Serial.begin(9600);
     
 // UV  VEML6070 SETUP ---------------
  uv.begin(VEML6070_4_T);  // pass in the integration time constant. max integration time/ per measurement (500msec)

 // RGB sensor SETUP ---------------
 if (RGB_sensor.init())
  { Serial.println("Sensor Initialization Successful\n\r");  }

 // VOCS sensor SETUP ---------------
if(!ccs.begin()){
    Serial.println("Failed to start sensor! Please check your wiring.");
   // while(1);
  }
  //calibrate temperature sensor
  while(!ccs.available());
  float temp = ccs.calculateTemperature();
  ccs.setTempOffset(temp - 25.0);


  //Radio SETUP ---------------

  pinMode(RFM95_RST, OUTPUT);
  digitalWrite(RFM95_RST, HIGH);

  //while (!Serial);
  Serial.begin(9600);
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
   // return;
  }
  else {
  Serial.println("initialization done.");
  }
  
 //Timers SETUP --------------- 
 beforeMillis = 0;
  t.every(650000, payload); //every when execute the function steilepayload
  
}
//^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^ VOID LOOP ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
void loop()
{
 
 
 // --------------  sequence of synchronisation when command t is received
 afterMillis = millis();
 duration = afterMillis - beforeMillis;
 
 if ((unsigned long)duration >= 6500) 
 {
  payload();
  beforeMillis = millis();
  t.update(); //update timer if we dont use it it bugs ---> millis().
    }
  
 // -------If any command was received... ------------
    
    uint8_t buf[RH_RF95_MAX_MESSAGE_LEN];
    uint8_t len = sizeof(buf);

    if (rf95.recv(buf, &len)) 
   {
           
      Serial.print("RSSI,");
      Serial.print(rf95.lastRssi(), DEC);Serial.print(",");
      Serial.println((char*)buf);

 /*     File dataFile = SD.open("datalog.txt", FILE_WRITE);
     // Everything received from the radio module is also saved on the sd card. In this  way we backup each modules data to both modules
     if (dataFile) {
    dataFile.println((char*)buf);
    dataFile.close();
    }   */
      memcpy(trig, &buf[0], 1 );//Acquire the first character of the buffer
      trig[2] = '\0';
   
    }
         if (strlen((char*)buf) > 0)
      { 
         if (strcmp_P((char*)trig, "t") == 0)    // If the first character of rxbuffer is t then you also send 
          { 
           delay(300);
           payload();
           beforeMillis = millis();  //Reset everything to zero
           }
        else if (strcmp_P((char*)buf, "Pcomtest") == 0)
          { 
          Pcomtest();
          }
         /*
         else if (strcmp_P((char*)buf, "entoli3") == 0)
          {
          
          }  */
         else if (strcmp_P((char*)buf, "PdumpSD") == 0)
          {
          PdumpSD();
          }
         else if (strcmp_P((char*)buf, "PdeleteSD") == 0)
          {
          PdeleteSD();
          }
         }
    void clearRxBuf(); //Clear receive buffer
    memset(buf, 0, sizeof(buf)); //Delete of variable buf

      //-----------------receive command from serial ----------------------

    while(Serial.available()) {     //As long as there is data in serial port
       Serial.readBytes(serialdata, 14); //We use readbytes as it used below for beaming to the CanSat,  Serial.readString is not converted
       Serial.println(serialdata);

    if (strcmp_P((char*)serialdata, "PdumpSD") == 0) {
      PdumpSD();
    }
      if (strcmp_P((char*)serialdata, "PdeleteSD") == 0){
      PdeleteSD();
    }
    if (strcmp_P((char*)serialdata, "Pcomtest") == 0){
      Pcomtest();
    }
    }  
    memset(serialdata, 0, sizeof(serialdata)); //Deletes the variable serialdata
 }



