void BNOvalues()
{

  //In this function we receive multiple acceleration data and we are able to keep the highest acceleration for beaming to the ground station
  //Every 5000ms it starts over. This way we recieve the greatest acceleration every 5 seconds provind us with data about the max stres our cansat had to survive. 

  sensors_event_t event;
  bno.getEvent(&event); //Receive data
  imu::Vector<3> acceleration = bno.getVector(Adafruit_BNO055::VECTOR_LINEARACCEL);//Receive acceleration values

  //   Serial.println(acceleration.x());
  float ax = acceleration.x();
  //  Serial.println(magnet.y());
  float ay = acceleration.y();
  //  Serial.println(magnet.z());
  float az = acceleration.z();


  //  accel = sqrt(pow(ax, 2) + pow(ay, 2) + pow(az, 2)); // σε m/s^2    We calculate the monometrical acceleration by fusing all 3 axes

  if (ax > maxaccX)  //If the value of acceleration is greater than the current one 
  {
    maxaccX = ax;    // Then the value maxACCEL  receives this value otherwise it remains unchanged
  }
  if (ay > maxaccY)  //If the value of acceleration is greater than the current one 
  {
    maxaccY = ay;    //Then the value maxACCEL  receives this value otherwise it remains unchanged
  }
  if (az > maxaccZ)  //If the value of acceleration is greater than the current one 
  {
    maxaccZ = az;    //Then the value maxACCEL  receives this value otherwise it remains unchanged
  }


  //  Serial.println(ax);
  // Serial.println(maxaccX);
}
