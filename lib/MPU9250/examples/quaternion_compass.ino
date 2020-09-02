/*
  "quaternion_compass_v1.ino"
  Code by LINGIB
  https://www.instructables.com/member/lingib/instructables/
  Last update 28 Nov 2019.

  ------
  About:
  ------
  This compass uses the "Sparkfun MPU-9250 Breakout Arduino Library",
  https://github.com/sparkfun/SparkFun_MPU-9250_Breakout_Arduino_Library, which was
  forked from https://github.com/kriswiner/MPU9250.

  This compass uses quaternions and behaves as though it is tilt-stabilized.

  The code uses the open source Madgwick and Mahony filter quaternion algorithms for calculating
  the pitch, roll, and yaw.

  The printNumber function uses two overloaded functions that keep your column-numbers
  steady, regardless of size or sign, by converting each number to a string then
  pre-padding each column-width with spaces. The column-widths, which are currently
  set to 6 may be altered by changing each "6" to "whatever-the-number-will-fit-into".

  ---------------
  Hardware setup:
  ---------------
  MPU9250 Breakout --------- Arduino
  VDD ---------------------- 5V
  SDA ---------------------- A4
  SCL ---------------------- A5
  GND ---------------------- GND

  External pull-ups are not required as the MPU9250 has
  internal 10K pull-ups to an internal 3.3V supply.

  The MPU9250 is an I2C sensor and uses the Arduino Wire
  library. Because the sensor is not 5V tolerant, the internal
  pull-ups used by the Wire library in the Wire.h/twi.c utility file.

  This may be achieved by editing lines 75,76,77 in your
  "C:\Users\Your_name\Documents\Arduino\libraries\Wire\utility\wire.c" file to read:

  // deactivate internal pullups for twi.
  digitalWrite(SDA, 0);
  digitalWrite(SCL, 0);

  ---------------
  Terms of use:
  ---------------
  The software is provided "AS IS", without any warranty of any kind, express or implied,
  including but not limited to the warranties of mechantability, fitness for a particular
  purpose and noninfringement. In no event shall the authors or copyright holders be liable
  for any claim, damages or other liability, whether in an action of contract, tort or
  otherwise, arising from, out of or in connection with the software or the use or other
  dealings in the software.

  -----------
  Warning:
  -----------
  Do NOT use this compass in situations involving safety to life
  such as navigation at sea.
*/

// ----- configure MPU9250
#include "quaternionFilters.h"
#include "MPU9250.h"
#define I2Cclock 400000                                 // I2C clock is 400 kilobits/s
#define I2Cport Wire                                    // I2C using Wire library
#define MPU9250_ADDRESS MPU9250_ADDRESS_AD0             // MPU9250 address when ADO = 0 (0x68)  
MPU9250 myIMU(MPU9250_ADDRESS, I2Cport, I2Cclock);      // Create myIMU instance using I2C at 400 kilobits/s

// ----- configure 16x2 LCD display
/* Comment-out the following line if you are not using a 16x2 LCD */
//#define LCD2
#ifdef LCD2
#include <LiquidCrystal_I2C.h>                                    // YwRobot Arduino LCM1602 IIC V1 library  
LiquidCrystal_I2C lcd(0x27, 2, 1, 0, 4, 5, 6, 7, 3, POSITIVE);    // LCD pinouts: addr,en,rw,rs,d4,d5,d6,d7,bl,blpol
#endif

// ----- Select a TASK
/* choose a TASK from the following list:
  #define TASK 0    // Calibrate each time at start-up
  #define TASK 1    // Calibrate once ... using onboard code
  #define TASK 2    // Calibrate once ... using external "compass_cal.pde" software
  #define TASK 3    // View ... register contents on "Serial Monitor" (115200 bauds)
  #define TASK 4    // View ... pitch, roll, and compass-heading on "Serial Monitor" (115200 bauds)
  #define TASK 5    // View ... compass-heading using external "compass_rose.pde" software
  #define TASK 6    // View ... pitch, roll, and compass-heading on 16x2 LCD display
*/
#define TASK 6

// ----- user offsets and scale-factors
/*
   Each of the following values must be overwritten with the offsets and scale-factors for
   YOUR location otherwise you will have to "tumble" your compass every time you switch it on.
   There are two methods for obtaining this data:

   Method 1:
   --------
   Set "#define TASK 1". Upload this change to your Arduino.
   You will be asked to "tumble" your  compass for 30 seconds.
   Replace (copy-&-paste) the values below with the offsets and scale-factors that appear on your computer screen.
   Once you have done this select one of  TASKs 3,4, or 5 and upload these changes to your Arduino
   This method is less accurate than Method 2

   Method 2:
   ---------
   Set "#define TASK 2". Upload this change to your Arduino.
   Run Processing "compass_cal.pde" and follow the on-screen instructions.
   Replace (copy-and-paste) the values below with the offsets and scale-factors that appear on your computer screen.
   Close Processing "compass_cal.pde"
   Once you have done this select one of  TASKs 3,4, or 5 and upload these changes to your Arduino
   This method is more accurate, and more consistent,than method 1
*/

// ----- NZ Offsets & Scale-factors
float
Mag_x_offset = -34.560013,
Mag_y_offset = 528.885,
Mag_z_offset = -125.259995,
Mag_x_scale = 1.0247924,
Mag_y_scale = 0.99078894,
Mag_z_scale = 0.9853226;

// ----- Magnetic declination
/*
  The magnetic declination for Lower Hutt, New Zealand is +22.5833 degrees
  Obtain your magnetic declination from http://www.magnetic-declination.com/
  By convention, declination is positive when magnetic north
  is east of true north, and negative when it is to the west.
  Substitute your magnetic declination for the "Declination" shown below.
*/
#define True_North false                                // change this to "true" for True North                
float Declination = +22.5833;                           // substitute your magnetic declination 

// ======================= NO CODE CHANGES ARE REQUIRED BEYOND THIS POINT =========================

// ----- Arduino pin definitions
short MyLed  = 13;                                      // LED on pin 13 ... toggles when data active

// ----- Processing variables
char InputChar;                                         // incoming characters stored here
bool LinkEstablished = false;                           // receive flag
String OutputString = "";                               // outgoing data string to Processing

// ----- software timer
unsigned long Timer1 = 500000L;                         // 500mS loop ... used when sending data to to Processing
unsigned long Stop1;                                    // Timer1 stops when micros() exceeds this value

// ----------
// setup()
// ----------
void setup()
{
  // ----- Enable comunications
  /* Don't forget to disable your I2C pull-up resistors */
  Serial.begin(115200);                                 // Start serial port
  while (!Serial);                                      // Not required for Arduino UNO but needed when porting code to the M4 Express
  Wire.begin();                                         // Start I2C as master
  Wire.setClock(400000);                                // Set I2C clock speed to 400kbs

  // ----- Configure LCD display
#ifdef LCD2
  lcd.begin(16, 2);                                     // Configure 16 char x 2 line LCD display
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print(F("   Quaternion"));
  lcd.setCursor(0, 1);
  lcd.print(F("   Compass V1"));
  delay(2000);
#endif

  // ----- LED indicator
  pinMode(MyLed, OUTPUT);
  digitalWrite(MyLed, HIGH);

  // ----- Look for MPU9250|MPU9255
  byte gyroID = myIMU.readByte(MPU9250_ADDRESS, WHO_AM_I_MPU9250);
  if (!((gyroID == 0x71) || (gyroID == 0x73)))

    /*
      I AM 73 means you have an MPU9255. More or less the same as
      MPU9250 but with some extra functionality.
    */

  {
    // ------ Failed to connect
    Serial.print(F("WHO_AM_I = "));
    Serial.println(gyroID, HEX);
    Serial.println(F("Could not connect to the MPU9250/MPU9255"));
    Serial.println(F("Communication failed ... program aborted !!"));
    Serial.flush();
    abort();
  }
  else
  {
    // ----- MPU9250|MPU9255 found
    if (gyroID == 0x71) Serial.println(F("WHO_AM_I = 0x71\nMPU9250 is online ..."));
    if (gyroID == 0x73) Serial.println(F("WHO_AM_I = 0x73\nMPU9255 is online ..."));
    Serial.println("");

    // ----- Start by performing self test and reporting values
    myIMU.MPU9250SelfTest(myIMU.selfTest);
    Serial.print(F("x-axis self test: acceleration trim within : "));
    Serial.print(myIMU.selfTest[0], 1); Serial.println("% of factory value");

    Serial.print(F("y-axis self test: acceleration trim within : "));
    Serial.print(myIMU.selfTest[1], 1); Serial.println("% of factory value");

    Serial.print(F("z-axis self test: acceleration trim within : "));
    Serial.print(myIMU.selfTest[2], 1); Serial.println("% of factory value");

    Serial.print(F("x-axis self test: gyration trim within : "));
    Serial.print(myIMU.selfTest[3], 1); Serial.println("% of factory value");

    Serial.print(F("y-axis self test: gyration trim within : "));
    Serial.print(myIMU.selfTest[4], 1); Serial.println("% of factory value");

    Serial.print(F("z-axis self test: gyration trim within : "));
    Serial.print(myIMU.selfTest[5], 1); Serial.println("% of factory value");
    Serial.println("");

    Serial.println(F("Place the compass on a level surface"));
    Serial.println(F(""));

#ifdef LCD2
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Place compass");
    lcd.setCursor(0, 1);
    lcd.print("on level surface");
#endif

    delay(4000);                        // allow time to place the compass in position

    // ----- Calibrate gyro and accelerometers, load biases in bias registers
    myIMU.calibrateMPU9250(myIMU.gyroBias, myIMU.accelBias);

    // ----- Initialize device for active mode read of accelerometer, gyroscope, and
    //       temperature
    myIMU.initMPU9250();
    Serial.println(F("MPU9250 initialized for active data mode...."));
    Serial.println("");
  }

  // ----- Look for the AK8963 magnetometer
  byte MagID = myIMU.readByte(AK8963_ADDRESS, WHO_AM_I_AK8963);
  if (!(MagID == 0x48))
  {
    // ----- Communication failed, stop here
    Serial.print(F("WHO_AM_I = "));
    Serial.println(MagID, HEX);
    Serial.println(F("Could not connect to the AK8963"));
    Serial.println(F("Communication failed ... program aborted !!"));
    Serial.flush();
    abort();
  }
  else
  {
    // ----- AK8963 found
    Serial.println(F("WHO_AM_I = 0x48\nAK8963 is online ..."));
    Serial.println("");

    // ----- Get factory ASA calibration values
    myIMU.initAK8963(myIMU.factoryMagCalibration);

    // ----- Initialize device for active mode read of magnetometer
    Serial.println(F("AK8963 initialized for active data mode...."));
    Serial.println("");

    // ----- Display AK8963 fuse rom values
    Serial.println(F("AK8963 Fuse ROM values: "));
    Serial.print(F("ASAX: "));
    Serial.println(myIMU.factoryMagCalibration[0], 2);
    Serial.print(F("ASAY: "));
    Serial.println(myIMU.factoryMagCalibration[1], 2);
    Serial.print(F("ASAZ: "));
    Serial.println(myIMU.factoryMagCalibration[2], 2);
    Serial.println("");

    // ----- Get correct sensor resolutions (You only need to do this once)
    myIMU.getAres();      // milli-gravity
    myIMU.getGres();      // dps
    myIMU.getMres();      // milli-Gauss 14-bit|16-bit

    // ---- display sensor scale multipliers
    Serial.println(F("Sensor-scale multipliers"));
    Serial.print(F("accel: "));
    Serial.println(myIMU.aRes, 6);
    Serial.print(F(" gyro: "));
    Serial.println(myIMU.gRes, 6);
    Serial.print(F("  mag: "));
    Serial.println(myIMU.mRes, 6);
    Serial.println("");
  }

  // -----------------------------------------------------------------
  // TASK 0,1 ... On-board calibration
  // -----------------------------------------------------------------
  if ((TASK == 0) || (TASK == 1))
  {
#ifdef LCD2
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print(F("Tumble compass"));
    lcd.setCursor(0, 1);
    lcd.print(F("for 30 seconds"));
#endif

    myIMU.magCalMPU9250(myIMU.magBias, myIMU.magScale);

#ifdef LCD2
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print(F("Stop tumbling"));
    delay(4000);
#endif
  }

  if (TASK == 1)
  {
#ifdef LCD2
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print(F("Record offsets"));
    lcd.setCursor(0, 1);
    lcd.print(F("& scale-factors"));
#endif

    // ----- Message
    Serial.println(F("------------------------------------------"));
    Serial.println(F("Copy-&-paste the following code into your "));
    Serial.println(F("Arduino header then delete the old code."));
    Serial.println(F("------------------------------------------"));
    Serial.println(F(""));
    Serial.println(F("float"));
    Serial.print(F("Mag_x_offset = "));
    Serial.print(myIMU.magBias[0]);
    Serial.println(",");
    Serial.print(F("Mag_y_offset = "));
    Serial.print(myIMU.magBias[1]);
    Serial.println(",");
    Serial.print(F("Mag_z_offset = "));
    Serial.print(myIMU.magBias[2]);
    Serial.println(",");
    Serial.print(F("Mag_x_scale = "));
    Serial.print(myIMU.magScale[0]);
    Serial.println(",");
    Serial.print(F("Mag_y_scale = "));
    Serial.print(myIMU.magScale[1]);
    Serial.println(",");
    Serial.print(F("Mag_z_scale = "));
    Serial.print(myIMU.magScale[2]);
    Serial.println(F(";"));

    // ----- Halt program
    while (true);
  }

  // -----------------------------------------------------------------
  // TASK 2 ... message
  // -----------------------------------------------------------------
  if (TASK == 2)
  {
#ifdef LCD2
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print(F("Run compass_cal"));
    lcd.setCursor(0, 1);
    lcd.print(F("on your PC"));
#endif
  }

  // -----------------------------------------------------------------
  // TASK 3,4,5,6 ... Use recorded offsets and scale factors
  // -----------------------------------------------------------------
  if ((TASK == 3) || (TASK == 4) || (TASK == 5) || (TASK == 6))
  {
    // ----- Copy the hard-iron offsets
    myIMU.magBias[0] = Mag_x_offset;
    myIMU.magBias[1] = Mag_y_offset;
    myIMU.magBias[2] = Mag_z_offset;

    // ----- Copy the soft-iron scalefactors
    myIMU.magScale[0] = Mag_x_scale;
    myIMU.magScale[1] = Mag_y_scale;
    myIMU.magScale[2] = Mag_z_scale;

    // ----- Display offsets & scale-factors
    Serial.println("");
    Serial.print("Mag_x_offset = ");
    Serial.println(myIMU.magBias[0]);
    Serial.print("Mag_y_offset = ");
    Serial.println(myIMU.magBias[1]);
    Serial.print("Mag_z_offset = ");
    Serial.println(myIMU.magBias[2]);
    Serial.print("Mag_x_scale = ");
    Serial.println(myIMU.magScale[0]);
    Serial.print("Mag_y_scale = ");
    Serial.println(myIMU.magScale[1]);
    Serial.print("Mag_z_scale = ");
    Serial.println(myIMU.magScale[2]);
    Serial.println("");
  }

  // -----------------------------------------------------------------
  // TASK 3,4 ... common message
  // -----------------------------------------------------------------
  if ((TASK == 3) || (TASK == 4))
  {
#ifdef LCD2
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print(F("Serial Monitor"));
    lcd.setCursor(0, 1);
    lcd.print(F("115200 bauds"));
#endif
  }

  // -----------------------------------------------------------------
  // TASK 5 ... message
  // -----------------------------------------------------------------
  if (TASK == 5)
  {
#ifdef LCD2
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print(F("Run compass_rose"));
    lcd.setCursor(0, 1);
    lcd.print(F("on your PC"));
#endif
  }

  // ----- start software timer
  Stop1 = micros() + Timer1;                            // Used by send_data() function
}

// ----------
// loop()
// ----------
void loop()
{
  refresh_data();                                       // this must be done each time through the loop

  // ----- Processing Tasks
  switch (TASK) {
    case 2:
      compass_cal();                                    // Get compass offsets and scale-factors using Processing "compass_cal.pde" (circle-method)
      break;
    case 5:
      compass_rose();                                   // Display compass heading using Processing "compass_rose.pde"
      break;
    default:
      break;
  }

  // ----- Perform these tasks every 500mS
  if (micros() > Stop1)
  {
    Stop1 += Timer1;                                    // Reset timer

    // ----- Serial Monitor Tasks
    switch (TASK) {
      case 0:
        display_compass_heading_on_lcd();               // Send data to LCD display
        break;
      case 3:
        display_register_contents_on_serial_monitor();  // Display MPU9250 register contents on Serial Monitor (115200 bauds)
        break;
      case 4:
        display_compass_heading_on_serial_monitor();    // Display compass pitch roll & heading on Serial Monitor (115200 bauds)
        break;
      case 6:
        display_compass_heading_on_lcd();               // Send data to LCD display
        break;
      default:
        break;
    }
  }
}
