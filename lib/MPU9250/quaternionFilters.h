/*
  This code is unchanged
  
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

#ifndef _QUATERNIONFILTERS_H_
#define _QUATERNIONFILTERS_H_

#include <Arduino.h>

// ============================================
void MadgwickQuaternionUpdate(float ax, float ay, float az,
                              float gx, float gy, float gz,
                              float mx, float my, float mz,
                              float deltat);

// ============================================
void MahonyQuaternionUpdate(float ax, float ay, float az,
                            float gx, float gy, float gz,
                            float mx, float my, float mz,
                            float deltat);

// ============================================
const float * getQ();

#endif // _QUATERNIONFILTERS_H_
