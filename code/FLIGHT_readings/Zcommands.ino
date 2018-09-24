void flightdata()  //  Is sent after  a command from  the groundstation  (flightdata)
{
  char ftmp1[8],ftmp2[8],tmp4[16],tmp5[16],ftmp3[8];

  ftoa(ftmp1, PIDstatus, 0);
  ftoa(ftmp2, maxAngle, 0);
  ftoa(tmp4, targetLat, 6);
  ftoa(tmp5, targetLon, 6);
  ftoa(ftmp3, startingAlt, 0);

  sprintf(fdata, "***** PID:%s,Ang:%s,LAT:%s,LON:%s,Alt:%s *****", ftmp1,ftmp2,tmp4,tmp5,ftmp3);

  digitalWrite(LED, HIGH);

  //  unsigned long beforeMillis = millis(); //Time stamp before beaming to groundstation

  //   rf95.send((uint8_t *)fdata, sizeof((uint8_t *)fdata)); // As long as it is NO SPACES ARE REQUIRED FOR THE DATA PACKET
  rf95.send((uint8_t *)fdata, 70);  // specified lenght of sent data - SAFER
  rf95.waitPacketSent();
  unsigned long afterMillis = millis();

  //  unsigned long duration = afterMillis - beforeMillis; //Time stamp after the beamingg to the groundstation
  //     Serial.print("send duration:");Serial.print(duration);Serial.println("ms");// Time required to beam (ms)

  digitalWrite(LED, LOW);
  Serial.println(fdata);

  fdata[0] = '\0';
  void clearRxBuf();
  delay(50);
}

void changepid()    //Changes flight scenario after sent command
{
  PIDstatus = atof(noletters);  //The value PIDst becomes equal to the convertion of noletters to float by array
  digitalWrite(LED, HIGH);
  rf95.send((uint8_t *)("PID_status"), 12);  //Send Pid_status
  rf95.waitPacketSent();
  rf95.send((uint8_t *)noletters, 3);  // Equal to noletters
  rf95.waitPacketSent();
  digitalWrite(LED, LOW);
  // Serial.println(PIDstatus);
  void clearRxBuf();
  delay(50);
}

void chAngle()  //Changes max servo pull angle after sent command
{
  maxAng = atof(noletters);  // The value maxAng  becomes equal to the convertion of noletters to float by array
  maxAngle = (int)maxAng;  // naxAngle is the int convertion of  maxAng 
  digitalWrite(LED, HIGH);
  rf95.send((uint8_t *)("maxAngle"), 11);  //Send maxAngle
  rf95.waitPacketSent();
  rf95.send((uint8_t *)noletters, 3);  // equal to noletters
  rf95.waitPacketSent();
  digitalWrite(LED, LOW);
  void clearRxBuf();
  delay(50);
  //  Serial.println(maxAngle);
}
void changelat()
{
  targetLat = atof(noletters);
  digitalWrite(LED, HIGH);
  rf95.send((uint8_t *)("New Lat"), 8);
  rf95.waitPacketSent();
  rf95.send((uint8_t *)noletters, 11);  // specified lenght of sent data - SAFER
  rf95.waitPacketSent();
  digitalWrite(LED, LOW);
  Serial.println(targetLat, 6 );
  void clearRxBuf();
  delay(50);
}
void changelon()
{
  targetLon = atof(noletters);
  digitalWrite(LED, HIGH);
  rf95.send((uint8_t *)("New Lon"), 8);
  rf95.waitPacketSent();
  rf95.send((uint8_t *)noletters, 11);  // specified lenght of sent data - SAFER
  rf95.waitPacketSent();
  digitalWrite(LED, LOW);
  Serial.println(targetLon, 6);
  void clearRxBuf();
  delay(50);
}
void changealt()
{
  targetAlt = atof(noletters);
  digitalWrite(LED, HIGH);
  rf95.send((uint8_t *)("New_t_Alt"), 10);
  rf95.waitPacketSent();
  rf95.send((uint8_t *)noletters, 11);  // specified lenght of sent data - SAFER
  rf95.waitPacketSent();
  digitalWrite(LED, LOW);
  Serial.println(targetAlt, 6);
  void clearRxBuf();
  delay(50);
}
void  changetimePID()
{
  digitalWrite(LED, HIGH);
  rf95.send((uint8_t *)("New_timing"), 12);
  rf95.waitPacketSent();
  rf95.send((uint8_t *)noletters, 11);  // specified lenght of sent data - SAFER
  rf95.waitPacketSent();
  digitalWrite(LED, LOW);
  Serial.println(interval);
  void clearRxBuf();
  delay(50);
}

void gpsA(void)
{
  digitalWrite(LED, HIGH);
  // rf95.send((uint8_t *)message1, 14);
  rf95.send((uint8_t *)("target_ligkiades"), 16);
  Serial.println("target_ligkiades");
  rf95.waitPacketSent();
  digitalWrite(LED, LOW);

  targetLat = 39.689147;
  targetLon = 20.889078;

  void clearRxBuf();
  delay(50);
}
void gpsB()
{
  digitalWrite(LED, HIGH);
  rf95.send((uint8_t *)("target_sxoleio"), 14);
  Serial.println("target_sxoleio");
  rf95.waitPacketSent();
  digitalWrite(LED, LOW);

  targetLat = 39.660363;
  targetLon = 20.851830;
  void clearRxBuf();
  delay(50);
}

void gpsC()
{
  digitalWrite(LED, HIGH);
  rf95.send((uint8_t *)("target_UOI"), 14);
  Serial.println("target_UOI");
  rf95.waitPacketSent();
  digitalWrite(LED, LOW);

  targetLat = 39.616238;
  targetLon = 20.844518;
  void clearRxBuf();
  delay(50);
}
void gpsD()
{
  digitalWrite(LED, HIGH);
  rf95.send((uint8_t *)("target_Marmara"), 16);
  Serial.println("target_Marmara");
  rf95.waitPacketSent();
  digitalWrite(LED, LOW);

  targetLat = 39.659761;
  targetLon = 20.799044;
  void clearRxBuf();
  delay(50);
}
void FdumpSD(void)
{
  digitalWrite(LED, HIGH);
  rf95.send((uint8_t *)message4, 13);
  rf95.waitPacketSent();
  digitalWrite(LED, LOW);
  File dataFile = SD.open("datalog.txt");
  // if the file is available, write to it:
  Serial.println("Start of FLIGHT SD dump");
  if (dataFile) {
    while (dataFile.available()) {
      Serial.write(dataFile.read());
    }
    dataFile.close();
  }
  Serial.println("End of FLIGHT SD dump");
  void clearRxBuf();
  delay(50);
}
void FdeleteSD(void)
{
  digitalWrite(LED, HIGH);
  rf95.send((uint8_t *)message5, 13);
  rf95.waitPacketSent();
  digitalWrite(LED, LOW);
  Serial.println("Removing datalog.txt...");
  SD.remove("datalog.txt");
  void clearRxBuf();
  delay(50);
}

void deleteGPS()
{
  Serial.println("clearing GPS logger");
  GPS.sendCommand("$PMTK184,1*22");  // Clears the logger
}

void dumpGPS()
{
  //GPS.sendCommand("$PMTK622,1*29");  // Dumps the GPS internal logger
}

