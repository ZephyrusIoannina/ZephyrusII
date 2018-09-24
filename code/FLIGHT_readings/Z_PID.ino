//============================functions related to autonomous flight==========================================

//---------------dataGPS-----------------------------
void pidfunction()
{
  if (GPS.fix) {

    // float fLat = decimalDegrees(GPS.latitude, GPS.lat); // Converting to DecimalDegrees with the use of a function in the end
    //  float fLon = decimalDegrees(GPS.longitude, GPS.lon);

    float fLat = GPS.latitudeDegrees; // Convertion to DecimalDegrees by the Gps module
    float fLon = GPS.longitudeDegrees;

    tripDistance = (double)calc_dist(fLat, fLon, targetLat, targetLon); //calculate distance

    if ((calc_bearing(fLat, fLon, targetLat, targetLon)) > 0) {       //calculate heading with function at the end.
      fHeading = (float)calc_bearing(fLat, fLon, targetLat, targetLon);
    }
    else {                                                            // For western hemisphere
      fHeading = (int)calc_bearing(fLat, fLon, targetLat, targetLon) + 360;
    }

    //  Serial.println(double(calc_bearing(fLat, fLon, targetLat, targetLon)));
    heading = int(fHeading);
    dist = long(tripDistance);
  }
  //descent rate calculation
  int drop = Apold - Ap; //calculate drop in 1 sec
  int droptime = Ap / drop; //calculate projected drop time
  float d = GPS.speed * 0.51444444444444;  //το GPS.speed is in  knots converted to m/second
  int Speed = int(d);
  int travel = Speed * droptime; //projected distance at this speed.

  //****Data received from BNO*****************
  sensors_event_t event;
  bno.getEvent(&event);
  uint8_t system, gyro, accel, mag;
  system = gyro = accel = mag = 0;
  bno.getCalibration(&system, &gyro, &accel, &mag);

  //Serial.println(system, DEC);//calibration value
  int calib = (system, DEC);
  //Serial.println(event.orientation.x, 4);//heading in Χ Axis

  //******Choise of absolute heading******

  float realheading; // real heading 
  if (calib < 1) // If the Bno sensor  is not yet fully calibrated (0 being bad and 3 perfect)
  {
    realheading = GPS.angle; // The value received is the angle  in relation to north received from the gps
  }
  else {
    realheading = event.orientation.x;// If bno is calibrated use the value from Bno sensor
  }

  //*****Sets the desired heading value for the target. (calculated by gps)************ 

  float headingDiff = fHeading - realheading;//headingdiff(Difference between desired and real heading) Difference between desired and real heading

  //If value is negative we must turn left  -------- If value is positive we must turn right


  if (headingDiff > 180) //If difference bigger than 180 degrees
    headingDiff = -360 + headingDiff; //We substract 360 to receive the smallest absolute number (+ or -) 
  if (headingDiff < -180) //εάν η διαφορά είναι μικρότερη από -180
    headingDiff = 360 + headingDiff; // We add 360 to receive the smallest absolute number (+ or -)
  // If nothing of the above is used the value remains unchanged
  //(Positive left, Negative right)

  //  headingDiff = 19 ;//testing
  // PIDstatus = 21; //testing

  /*  if (-181 < testDiff && testDiff < 179)      //testing
    {
      testDiff = testDiff + 5; //testing
    }
    else
    { testDiff = -180 ;
    }  */



  // headingDiff = testDiff; //testing
  int intDiff = (int)headingDiff; // convert float to int for calculations


  // Serial.println(PIDstatus);
  // Serial.println(maxAngle);

  if (PIDstatus == 31)
  {
    //max range protection 
    if (travel > 700)
    {
      myservoL.write(5); //left servo not pulling
      myservoR.write(55);
      }
    else{  
    if (20 < headingDiff && headingDiff < 180) //Have turn right
    {
      myservoL.write(5); //left servo not pulling
      myservoR.write(115 - maxAngle);
    }
    if (0 < headingDiff && headingDiff < 19) //bellow 20 degrees
    {
      myservoL.write(5); //Nothing
      myservoR.write(115);
    }

    if (-20 > headingDiff && headingDiff > -180) // Have to turn left
    {
      myservoL.write(5 + maxAngle);
      myservoR.write(115);//Right servo not pulling
    }
    if (0 > headingDiff && headingDiff > -19) //Bellow 20 degrees
    {
      myservoL.write(5);
      myservoR.write(115);//Noting
    }
  }
  }
  else if (PIDstatus == 24)
  {
    //on off autoturning with +-20 no action angle
    if (20 < headingDiff && headingDiff < 180) //Want to turn right
    {
      myservoL.write(5); //Left servo not pulling
      myservoR.write(115 - maxAngle);
    }
    if (0 < headingDiff && headingDiff < 19) //Bellow 20 degrees
    {
      myservoL.write(5); //Nothing
      myservoR.write(115);
    }

    if (-20 > headingDiff && headingDiff > -180) //Want to turn right
    {
      myservoL.write(5 + maxAngle);
      myservoR.write(115);//Left serbo not pulling
    }
    if (0 > headingDiff && headingDiff > -19) //Bellon 20 degrees
    {
      myservoL.write(5);
      myservoR.write(115);//Nothing
    }
  }
  else if (PIDstatus == 23)
  {
    //analog autoturning with bigger initial starting angle
    int turn; //How much it will turn
    turn = (maxAngle - 50) * abs(intDiff) / 180; //Same as max angle multiplied by diversion( without a negative or positive sign) devided by 180. 
    //Serial.println(turn);

    if (20 < headingDiff && headingDiff < 180) //Want to turn right
    {
      myservoL.write(5); //Left servo is not pulling
      myservoR.write(65 - turn);
    }
    if (0 < headingDiff && headingDiff < 19) //Want to turn right
    {
      myservoL.write(5); //Left servo not pulling
      myservoR.write(95);
    }

    if (-20 > headingDiff && headingDiff > -180) //Want to turn right
    {
      myservoL.write(55 + turn);
      myservoR.write(115);//Right servo not pulling
    }
    if (0 > headingDiff && headingDiff > -19) //Want to turn right
    {
      myservoL.write(25);
      myservoR.write(115);//Right servo not pulling
    }
  }
  else if (PIDstatus == 22)
  {
    //Analog autoturning
    int turn; //πόσο θα στρίψει
    turn = maxAngle * abs(intDiff) / 180;  //Same as max angle multiplied by diversion( without a negative or positive sign) devided by 180.
    Serial.println(turn);

    if (0 < headingDiff && headingDiff < 180) //Want to turn right
    {
      myservoL.write(5); //Left servo not pulling
      myservoR.write(115 - turn);
    }

    if (0 > headingDiff && headingDiff > -180) //Want to turn right
    {
      myservoL.write(5 + turn);
      myservoR.write(115);//Right servo not pulling
    }
  }
  else if (PIDstatus == 21)
  {
    //autoturning no PID

    if (0 < headingDiff && headingDiff < 30) //Want to turn right
    {
      myservoL.write(5); //Left servo not pulling
      myservoR.write(115 - maxAngle / 4);
    }
    if (30 < headingDiff && headingDiff < 90) //Want to turn right
    {
      myservoL.write(5); //Left servo not pulling
      myservoR.write(115 - maxAngle / 2);
    }
    if (90 < headingDiff && headingDiff < 180) //Want to turn right
    {
      myservoL.write(5); //Left servo not pulling
      myservoR.write(115 - maxAngle);
    }

    if (0 > headingDiff && headingDiff > -30) //Want to turn right
    {
      myservoL.write(5 + maxAngle / 4);
      myservoR.write(115);//Right servo not pulling
    }
    if (-30 > headingDiff && headingDiff > -90) //Want to turn right
    {
      myservoL.write(5 + maxAngle / 2);
      myservoR.write(115);//Right servo not pulling
    }
    if (-90 > headingDiff && headingDiff > -180) //Want to turn right
    {
      myservoL.write(5 + maxAngle);
      myservoR.write(115);//Right servo not pulling
    }
  }
  else if (PIDstatus == 20)
  {
    //Simple left or right turn in max angle

    if (0 < headingDiff && headingDiff < 180) //Want to turn right
    {
      myservoL.write(5); //Left servo not pulling
      myservoR.write(115 - maxAngle);
    }


    if (0 > headingDiff && headingDiff > -180) //Want to turn right
    {
      myservoL.write(5 + maxAngle);
      myservoR.write(115);//Right servo not pulling
    }

  }
  else if (PIDstatus == 1)
  {
    //zenith
    myservoR.write(115);

    myservoL.write(5);
  }
  else if (PIDstatus == 2)
  {
    //right full maxAngle
    myservoR.write(115 - maxAngle);

    myservoL.write(5);

  }
  else if (PIDstatus == 3)
  {
    //left full maxAngle
    myservoR.write(115);

    myservoL.write(5 + maxAngle);
  }

  else if (PIDstatus == 4)
  {
    // full maxAngle
    myservoR.write(115 - maxAngle);

    myservoL.write(5 + maxAngle);
  }

  else if (PIDstatus == 10)

    //κανονική PID
  {
    Input = abs(headingDiff); // The value that will be changed (input) is the difference between Real heading and required heading
    Setpoint = 0; //Has to become same as setpoit ( 0)
    //***Change of parameters depending on deviation ***
    double gap = abs(Setpoint - Input); //distance away from setpoint
    if (gap < 45)
    { //we're close to setpoint, use conservative tuning parameters
      myPID.SetTunings(consKp, consKi, consKd);
    }
    else
    {
      //we're far from setpoint, use aggressive tuning parameters
      myPID.SetTunings(aggKp, aggKi, aggKd);
    }
    myPID.Compute();
    Serial.println(Input, 4);//The pid input - Angle difference
    Serial.println(Setpoint, 4);// How much we want them to become -0
    Serial.println(Output, 4);//The value that gives us an answer of 0-255

    int turnangle;// Angle that our sattelite has to turn which is a product of the pid output with the servo available turning angle
    turnangle = map(Output, 0, 255, 0, maxAngle); //We set a value from 0-255 to the max angle (maxAngle)

    if (0 < headingDiff && headingDiff < 180) //Want to turn right
    {
      myservoL.write(5); //Left servo is not pulling
      myservoR.write(115 - turnangle);//Turns as much as pid tells it to
    }


    if (0 > headingDiff && headingDiff > -180) //Want to turn right
    {
      myservoL.write(5 + turnangle);//Turns as much as pid tells it to
      myservoR.write(115);//Right servo is not pulling
    }
  }
}

