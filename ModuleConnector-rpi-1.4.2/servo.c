/*
 * servo.c:
 *	Test of the softServo code.
 *	Do not use this code - use the servoBlaster kernel module instead
 *
 * Copyright (c) 2012-2013 Gordon Henderson. <projects@drogon.net>
 ***********************************************************************
 * This file is part of wiringPi:
 *	https://projects.drogon.net/raspberry-pi/wiringpi/
 *
 *    wiringPi is free software: you can redistribute it and/or modify
 *    it under the terms of the GNU Lesser General Public License as published by
 *    the Free Software Foundation, either version 3 of the License, or
 *    (at your option) any later version.
 *
 *    wiringPi is distributed in the hope that it will be useful,
 *    but WITHOUT ANY WARRANTY; without even the implied warranty of
 *    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *    GNU Lesser General Public License for more details.
 *
 *    You should have received a copy of the GNU Lesser General Public License
 *    along with wiringPi.  If not, see <http://www.gnu.org/licenses/>.
 ***********************************************************************
 */

#include <stdio.h>
#include <errno.h>
#include <string.h>

#include <wiringPi.h>
#include <softPwm.h>
#define pwmpin  0
#define pwrpin  3
int main ()
{
  wiringPiSetup ();
  softPwmCreate (pwmpin, 0, 200);
  pinMode (pwrpin, OUTPUT) ;

  digitalWrite(pwrpin, HIGH);
  delay(100);
  for (int i=0; i<3; i++){
    softPwmWrite (pwmpin, 6);
    delay(2000);  
    softPwmWrite (pwmpin, 25);
    delay(2000);
  }
  
  digitalWrite(pwrpin, LOW);
  delay(100);
  for (int i=0; i<3; i++){
    softPwmWrite (pwmpin, 6);
    delay(2000);  
    softPwmWrite (pwmpin, 25);
    delay(2000);
  }
 

}
