void functiontemplate()
{
  digitalWrite(LED, HIGH);
  rf95.send((uint8_t *)message2, 14);
  rf95.waitPacketSent();
  digitalWrite(LED, LOW);
}
void PdumpSD(void)
{
  digitalWrite(LED, HIGH);
  rf95.send((uint8_t *)message4, 14);
  rf95.waitPacketSent();
  digitalWrite(LED, LOW);
     File dataFile = SD.open("datalog.txt");
  // if the file is available, write to it:
  Serial.println("Start of PAYLOAD SD dump");
  if (dataFile) {
    while (dataFile.available()) {
      Serial.write(dataFile.read());
    }
    dataFile.close();
  }
  Serial.println("End of PAYLOAD SD dump");
}
void PdeleteSD(void)
{
  digitalWrite(LED, HIGH);
  rf95.send((uint8_t *)message5, 14);
  rf95.waitPacketSent();
  digitalWrite(LED, LOW);
  Serial.println("Removing datalog.txt...");
  SD.remove("datalog.txt");
}
void Pcomtest(void)
{
  digitalWrite(LED, HIGH);
 // rf95.send((uint8_t *)message1, 14);
  rf95.send((uint8_t *)("Payload_com_ok"), 16);
  Serial.println("Payload_com_ok");
  rf95.waitPacketSent();
  digitalWrite(LED, LOW);
}

//command to convert double to string 
char *ftoa(char *buffer, double d, int precision) {
  long wholePart = (long) d;
  // Deposit the whole part of the number.
  itoa(wholePart,buffer,10);
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

