 void steiledata()

{
  //********* BME280 values
  T = bme.readTemperature();
  P = bme.readPressure() / 100.0F;
  Apold = Ap; //keep older value for projections;
  Ap = bme.readAltitude(SEALEVELPRESSURE_HPA);
  H = bme.readHumidity();

  /* if (GPS.fix) {
     Serial.print(GPS.hour, DEC); Serial.print(':');
     Serial.print(GPS.minute, DEC); Serial.print(':');
     Serial.print(GPS.seconds, DEC); Serial.print('.');
     Serial.print("Location: ");
     Serial.print(GPS.latitude, 4); Serial.println(GPS.lat);
     Serial.print(", ");
     Serial.print(GPS.longitude, 4); Serial.println(GPS.lon);
     Serial.print("Altitude: "); Serial.println(GPS.altitude);
     Serial.print("Speed (knots): "); Serial.println(GPS.speed);
     Serial.print("Altitude: "); Serial.println(GPS.altitude);    }*/

  //  Serial.print("Satellites: "); Serial.println((int)GPS.satellites);


  //************Transform Data
  char tmp1[16], tmp2[16];
  float a = GPS.latitudeDegrees;
  float b = GPS.longitudeDegrees;


  ftoa(tmp1, a, 6);  // If more accuracy is required we change 6 to 10 and also increase the tmp1,2 correspondingly
  ftoa(tmp2, b, 6);

  // Hour  ,  Minute ,   Second      Temperature, Pressure, humidity ,Height(pressure),Longitude,latitude
  sprintf(apostoli, ",%d,%d,%d,%d,%d,%d,%d,%s,%s", GPS.hour, GPS.minute, GPS.seconds, (int)T, (int)P, (int)H, (int)Ap, tmp1, tmp2); //9 values

  digitalWrite(LED, HIGH);

  //   rf95.send((uint8_t *)apostoli, sizeof((uint8_t *)apostoli)); //As long as it is NO SPACES ARE REQUIRED FOR THE DATA PACKET
  rf95.send((uint8_t *)apostoli, 44);  // specified lenght of sent data - SAFER
  rf95.waitPacketSent();

  digitalWrite(LED, LOW);
  Serial.println(apostoli);

  //write to card
  File dataFile = SD.open("datalog.txt", FILE_WRITE);
  // if the file is available, write to it:
  if (dataFile) {
    dataFile.println(apostoli);
    dataFile.close();
    // print to the serial port too:
    // Serial.println(apostoli);
  }
  // if the file isn't open, pop up an error:
  else {
    Serial.println("error opening datalog.txt");
  }
  apostoli[0] = '\0';
}

void steiledataB()
{
  //μαγνητικά πεδία ------------------------------------------------------------------------
  
  delay(50);
  sensors_event_t event;
  bno.getEvent(&event); //Receive data

  imu::Vector<3> magnet = bno.getVector(Adafruit_BNO055::VECTOR_MAGNETOMETER);//Receive magnetometer values
  imu::Vector<3> gravity = bno.getVector(Adafruit_BNO055::VECTOR_GRAVITY);//Receive magnetometer values

  uint8_t system, gyro, accel, mag = 0;
  // system = gyro = accel = mag = 0;
  bno.getCalibration(&system, &gyro, &accel, &mag);//Recive value of calibration (0-3)
  //Serial.println(system);//calibration value


  int8_t temp = bno.getTemp();
  // Serial.print("Tbno: ");
  // Serial.println(temp);

  //----------------- Magnetic values

  //  Serial.println(magnet.x());
  int mx = (int)magnet.x();
  //  Serial.println(magnet.y());
  int my = (int)magnet.y();
  //  Serial.println(magnet.z());
  int mz = (int)magnet.z();



  // ---------------Gravitational values

  // Serial.println(gravity.x());
  float gx = 1000 * gravity.x();
  // Serial.println(gravity.y());
  float gy = 1000 * gravity.y();
  // Serial.println(gravity.z());
  float gz = 1000 * gravity.z();

  float grav = (sqrt(pow(gx, 2) + pow(gy, 2) + pow(gz, 2))); // In μm/sec2
  int gravfield  = (int)grav;

  //---------------------bunny
  //  Serial.println(event.orientation.x);
  int ex = (int)event.orientation.x;
  //  Serial.println(event.orientation.y);
  int ey = (int)event.orientation.y;
  //  Serial.println(event.orientation.z);
  int ez = (int)event.orientation.z;

  //*********Battery
  float measuredvbat = analogRead(VBATPIN);
  measuredvbat *= 2;    // we divided by 2, so multiply back
  measuredvbat *= 3.3;  // Multiply by 3.3V, our reference voltage
  measuredvbat /= 1024; // convert to voltage
  // Serial.print("VBat: " ); Serial.println(measuredvbat);

  //************Transform Data
  char tmpe[8];

  int galt = (int)GPS.altitude;
  float d = GPS.speed * 0.51444444444444 * 100;  // GPS.speed in measured in knots and is converted to cm/second
  long Speed = long(d);
  float e = measuredvbat;

  //If more accuracy is required we change 6 to 10 and also increase the tmp1,2 correspondingly
  int ACCX = (int)maxaccX; //Accelerations
  int ACCY = (int)maxaccY;
  int ACCZ = (int)maxaccZ;
  ftoa(tmpe, e, 3); //Voltage
  //  ftoa(acc, maxACCEL, 2); //Accelerations
  //gravitational      , Magnetometer x,y,z , temperature ,Χ, Υ, Ζ , distance , required heading, height(gps),   Speed(GPS)  , bat.voltage
  sprintf(apostoliB, "t,,,,,,,,,,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%s", (int)system, (long)gravfield, (int)mx, (int)my, (int)mz, (int)ACCX, (int)ACCY, (int)ACCZ, (int)temp, (int)ex, (int)ey, (int)ez, (long)dist, (int)heading, (int)galt, (long)Speed, tmpe); //14τιμές

  digitalWrite(LED, HIGH);
  unsigned long beforeMillis = millis(); //Time stamp before beaming to ground station
  //  rf95.send((uint8_t *)apostoliB, sizeof((uint8_t *)apostoliB)); //As long as it is NO SPACES ARE REQUIRED FOR THE DATA PACKET
  rf95.send((uint8_t *)apostoliB, 120);  // specified lenght of sent data - SAFER
  rf95.waitPacketSent();

  digitalWrite(LED, LOW);
  Serial.println(apostoliB);

  //write to card
  File dataFile = SD.open("datalog.txt", FILE_WRITE);
  // if the file is available, write to it:
  if (dataFile) {
    dataFile.println(apostoliB);
    dataFile.close();
  }
  // if the file isn't open, pop up an error:
  else {
    Serial.println("error opening datalog.txt");
  }
  maxaccX = 0; // We set the max acceleration to 0 before every new circle of calculations
  maxaccY = 0;
  maxaccZ = 0;

  apostoliB[0] = '\0';
}


