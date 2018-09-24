//============================Payload separate functions ==========================================

//---------------steile payload-----------------------------
void payload()
{
        //*********UV
    UV = uv.readUV();
    // Read sensor values (16 bit integers)
  unsigned int red = RGB_sensor.readRed();
  unsigned int green = RGB_sensor.readGreen();
  unsigned int blue = RGB_sensor.readBlue();
        //*********RGB
  // Print out readings, change HEX to DEC if you prefer decimal output
/*  Serial.print("Red: "); Serial.println(red,DEC);
  Serial.print("Green: "); Serial.println(green,DEC);
  Serial.print("Blue: "); Serial.println(blue,DEC);*/

  int Lred = (int)red;
  int Lgreen = (int)green;
  int Lblue = (int)blue;
    
       //*********VOCs
    if(ccs.available()){
     temp = ccs.calculateTemperature();
    }
    if(!ccs.readData()){
     /* Serial.print("CO2: ");
      Serial.print(ccs.geteCO2());
      Serial.print("ppm, TVOC: ");
      Serial.print(ccs.getTVOC());
      Serial.print("ppb   Temp:");
      Serial.println(temp);*/
    }
    else{
      Serial.println("vocs ERROR!");
     // while(1);
    }

     //*********Battery
     float measuredvbat = analogRead(VBATPIN);
     measuredvbat *= 2;    // we divided by 2, so multiply back
     measuredvbat *= 3.3;  // Multiply by 3.3V, our reference voltage
     measuredvbat /= 1024; // convert to voltage
    // Serial.print("VBat: " ); Serial.println(measuredvbat);

//************Transform Data

char tmp1[16], tmp2[16], tmp3[16];
 float a = ccs.geteCO2();  
 float b = ccs.getTVOC();
 float c = measuredvbat;
 
 ftoa(tmp1, a, 0);  //Αν θέλει περισσότερη ακρίβεια μεγαλώνουμε τα tmp ανάλογα If more accuracy is required we increase the tmp values correspondingly
 ftoa(tmp2, b, 0);
 ftoa(tmp3, c, 3);
 // sprintf(apostoliC,"PAYLOAD--UV:%d,R:%u,G:%u,B:%u,CO2:%s,VOC:%s,T:%d,%s",(int)UV,int(Lred),int(Lgreen),int(Lblue), tmp1, tmp2, int(temp), tmp3);
   sprintf(apostoliC,",,,,,,,,,,,,,,,,,,,,,,,,,,,%d,%u,%u,%u,%s,%s,%d,%s",(int)UV,int(Lred),int(Lgreen),int(Lblue),tmp1,tmp2,int(temp),tmp3);  //Used for live update 8 values
      
      digitalWrite(LED, HIGH);
   // rf95.send((uint8_t *)apostoliC, sizeof((uint8_t *)apostoli)); //As long as it is NO SPACES ARE REQUIRED FOR THE DATA PACKET
      rf95.send((uint8_t *)apostoliC, 60);  // specified lenght of sent data - SAFER    
      rf95.waitPacketSent();
      digitalWrite(LED, LOW);
      Serial.println(apostoliC);
     
     //write to card
      File dataFile = SD.open("datalog.txt", FILE_WRITE);
  // if the file is available, write to it:
  if (dataFile) {
    dataFile.println(apostoliC);
    dataFile.close();
    // print to the serial port too:
   // Serial.println(apostoli);
  }
  // if the file isn't open, pop up an error:
  else {
    Serial.println("error opening datalog.txt");
}  
   apostoliC[0]='\0';
  
}
