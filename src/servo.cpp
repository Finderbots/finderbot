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

#include <finderbot/servo.h>

int servo_setup()
{
  wiringPiSetup ();
  softPwmCreate (pwmpin, 0, 200);
  pinMode (pwrpin, OUTPUT);
  digitalWrite(pwrpin, LOW);
  softPwmWrite (pwmpin, 25);
  delay(1200);
  digitalWrite(pwrpin, HIGH);
  return 0;
}

int servo_turn0(){
  digitalWrite(pwrpin, LOW);
  softPwmWrite (pwmpin, 6);
  delay(1200);
  digitalWrite(pwrpin, HIGH);
  return 0;
}

int servo_turn90(){
  digitalWrite(pwrpin, LOW);
  softPwmWrite (pwmpin, 16);
  delay(1200);
  digitalWrite(pwrpin, HIGH);
  return 0;
}

int servo_turn180(){
  digitalWrite(pwrpin, LOW);
  softPwmWrite (pwmpin, 25);
  delay(1200);
  digitalWrite(pwrpin, HIGH);
  return 0;
}

// int main(){
// 	servo_setup();
// 	delay(2000);
// 	servo_turn0();
// 	delay(2000);
// 	servo_turn180();
// 	delay(2000);
// 	servo_turn0();
// 	delay(2000);
// 	servo_turn180();
// 	return 0;
// }

