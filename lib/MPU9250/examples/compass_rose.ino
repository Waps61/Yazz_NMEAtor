/*
  "compass_rose.ino"
  (C) 28 November 2019, by LINGIB
  https://www.instructables.com/member/lingib/instructables/

  The following code sends the compass-heading to Processing
  for controlling a graphics "compass rose" on your PC screen.

  ---------------
  Terms of use:
  ---------------
  The software is provided "AS IS", without any warranty of any kind, express or implied,
  including but not limited to the warranties of mechantability, fitness for a particular
  purpose and noninfringement. In no event shall the authors or copyright holders be liable
  for any claim, damages or other liability, whether in an action of contract, tort or
  otherwise, arising from, out of or in connection with the software or the use or other
  dealings in the software.
*/

void compass_rose()
{
  // ----- read input character
  if (Serial.available()) {
    InputChar = Serial.read();
    if ((InputChar == 's') || (InputChar == 'S')) {
      LinkEstablished = true;
    }
  }

  // ----- get compass heading
  /*
     The normal range for myIMU.yaw is  +/- 0..179 degrees. Adding 360 to
     the negative readings converts the range to 0..360 degrees. The yaw
     output is heavily damped but doesn't appear to suffer from pitch or roll.
     For all practical purposes the compass yaw and compass heading are the same
  */
  float heading = myIMU.yaw * RAD_TO_DEG;
  if (heading < 0) heading += 360.0;              // Yaw goes negative between 180 amd 360 degrees
  if (True_North == true) heading += Declination; // Calculate True North
  if (heading < 0) heading += 360.0;              // Keep heading within 0..360 degrees
  if (heading > 360) heading -= 360.0;

  // ------ create output data string
  OutputString = String(heading);

  // ----- send string if link established
  if (LinkEstablished && ((InputChar == 's') || (InputChar == 'S'))) {
    Serial.println(OutputString);
  }

  // ----- send 'S' if link not established
  if (micros() > Stop1) {
    Stop1 += Timer1;
    if (!LinkEstablished) {
      Serial.println('S');
    }
  }
}
