//command to convert double to string --------------
char *ftoa(char *buffer, double d, int precision) {
  long wholePart = (long) d;
  // Deposit the whole part of the number.
  itoa(wholePart, buffer, 10);
  // Now work on the faction if we need one.
  if (precision > 0) {
    // We do, so locate the end of the string and insert
    // a decimal point.
    char *endOfString = buffer;
    while (*endOfString != '\0') endOfString++;
    *endOfString++ = '.';
    // Now work on the fraction, be sure to turn any negative
    // values positive.
    if (d < 0) {
      d *= -1;
      wholePart *= -1;
    }
    double fraction = d - wholePart;
    while (precision > 0) {

      // Multipleby ten and pull out the digit.

      fraction *= 10;
      wholePart = (long) fraction;
      *endOfString++ = '0' + wholePart;

      // Update the fraction and move on to the
      // next digit.

      fraction -= wholePart;
      precision--;
    }

    // Terminate the string.

    *endOfString = '\0';
  }
  return buffer;
}
// GPS calculations commands--------------

//calculate bearing

int calc_bearing(float flat1, float flon1, float flat2, float flon2)
{
  float calc;
  float bear_calc;

  float x = 69.1 * (flat2 - flat1);
  float y = 69.1 * (flon2 - flon1) * cos(flat1 / 57.3);

  calc = atan2(y, x);

  bear_calc = degrees(calc);

  if (bear_calc <= 1) {
    bear_calc = 360 + bear_calc;
  }
  return bear_calc;
}

//calculate distance
unsigned long calc_dist(float flat1, float flon1, float flat2, float flon2)
{
  float dist_calc = 0;
  float dist_calc2 = 0;
  float diflat = 0;
  float diflon = 0;

  diflat = radians(flat2 - flat1);
  flat1 = radians(flat1);
  flat2 = radians(flat2);
  diflon = radians((flon2) - (flon1));

  dist_calc = (sin(diflat / 2.0) * sin(diflat / 2.0));
  dist_calc2 = cos(flat1);
  dist_calc2 *= cos(flat2);
  dist_calc2 *= sin(diflon / 2.0);
  dist_calc2 *= sin(diflon / 2.0);
  dist_calc += dist_calc2;

  dist_calc = (2 * atan2(sqrt(dist_calc), sqrt(1.0 - dist_calc)));

  dist_calc *= 6371000.0; //Converting to meters
  return dist_calc;
}

// Convert NMEA coordinate to decimal degrees
float decimalDegrees(float nmeaCoord, char dir) {
  uint16_t wholeDegrees = 0.01 * nmeaCoord;
  int modifier = 1;

  if (dir == 'W' || dir == 'S') {
    modifier = -1;
  }

  return (wholeDegrees + (nmeaCoord - 100.0 * wholeDegrees) / 60.0) * modifier;
}

