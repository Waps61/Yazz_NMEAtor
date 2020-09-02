/*
  "lcd_display.ino"
  (C) 28 November 2019, by LINGIB
  https://www.instructables.com/member/lingib/instructables/

  This routine displays the (P)itch, (R)oll, and compass Heading on a two line LCD display.  
  Should you have a different display, replace all references to LCD2 with your own code.
  
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

void display_compass_heading_on_lcd() {

  // ----- locals
  float pitch = myIMU.pitch * RAD_TO_DEG;
  float roll = myIMU.roll * RAD_TO_DEG;
  
  /*
       For all practical purposes the compass yaw and compass heading are the same
  */
  float heading = myIMU.yaw * RAD_TO_DEG;
  if (heading < 0) heading += 360.0;              // Yaw goes negative between 180 amd 360 degrees
  if (True_North == true) heading += Declination; // Calculate True North
  if (heading < 0) heading += 360.0;              // Keep heading within 0..360 degrees
  if (heading > 360) heading -= 360.0;

#ifdef LCD2
  lcd.clear();

  /*
     The pitch and roll readings are fixed width.
     This code can be shortened if you need more processing time
  */

  // ----- display the pitch
  lcd.setCursor(0, 1);
  (pitch < 0) ? lcd.print("P-") : lcd.print("P+");
  lcd.print((int(abs(pitch * 10)) / 1000));                // 1st digit
  lcd.print((int(abs(pitch * 10)) / 100) % 10);            // 2nd digit
  lcd.print((int(abs(pitch * 10)) / 10) % 10);             // 3rd digit
  lcd.print(".");                                          // decimal point
  lcd.print(int(abs(pitch * 10)) % 10);                    // last digit

  // ----- display the roll
  lcd.setCursor(9, 1);
  (roll < 0) ? lcd.print("R-") : lcd.print("R+");
  lcd.print((int(abs(roll * 10)) / 1000));                 // 1st digit
  lcd.print((int(abs(roll * 10)) / 100) % 10);             // 2nd digit
  lcd.print((int(abs(roll * 10)) / 10) % 10);              // 3rd digit
  lcd.print(".");                                          // decimal point
  lcd.print(int(abs(roll * 10)) % 10);                     // last digit

  // ----- display the heading
  lcd.setCursor(3, 0);
  lcd.print("Heading ");                                   // Heading
  lcd.print(round(heading));
#endif
}
