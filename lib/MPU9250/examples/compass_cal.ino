/*
  "compass_cal.ino"
  (C) 28 November 2019, by LINGIB
  https://www.instructables.com/member/lingib/instructables/

  The following code sends the magnetometer x|y|z data to Processing "compass_data_in.pde" 
  for analysis. Processing then outputs recommended values for your magnetometer offsets 
  & scalefactors. Copy these values into your header and set "#define CALIBRATED true"

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

void compass_cal()
{
  // ----- Locals
  float
  xPos,
  yPos,
  zPos;

  // ----- read input character
  if (Serial.available()) {
    InputChar = Serial.read();
    if ((InputChar == 's') || (InputChar == 'S')) {
      LinkEstablished = true;
    }
  }

  // -----Read the magnetometer x|y|z register values
  myIMU.readMagData(myIMU.magCount);

  // ----- calculate the magnetometer values (no offsets)
  xPos = (float)myIMU.magCount[0] * myIMU.factoryMagCalibration[0] * myIMU.mRes;   // raw mx * ASAX * 0.6 (mG)
  yPos = (float)myIMU.magCount[1] * myIMU.factoryMagCalibration[1] * myIMU.mRes;   // raw my * ASAY * 0.6 (mG)
  zPos = (float)myIMU.magCount[2] * myIMU.factoryMagCalibration[2] * myIMU.mRes;   // raw mz * ASAZ * 0.6 (mG)

  // ------ create output data string
  OutputString = String(xPos) + ',' + String(yPos) + ',' + String(zPos);

  // ----- send string if link established
  if (LinkEstablished && ((InputChar == 's') || (InputChar == 'S'))) {
    InputChar = 'x';
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
