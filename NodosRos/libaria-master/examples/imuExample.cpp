/*
Adept MobileRobots Robotics Interface for Applications (ARIA)
Copyright (C) 2004, 2005 ActivMedia Robotics LLC
Copyright (C) 2006, 2007, 2008, 2009, 2010 MobileRobots Inc.
Copyright (C) 2011, 2012, 2013 Adept Technology

     This program is free software; you can redistribute it and/or modify
     it under the terms of the GNU General Public License as published by
     the Free Software Foundation; either version 2 of the License, or
     (at your option) any later version.

     This program is distributed in the hope that it will be useful,
     but WITHOUT ANY WARRANTY; without even the implied warranty of
     MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
     GNU General Public License for more details.

     You should have received a copy of the GNU General Public License
     along with this program; if not, write to the Free Software
     Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA

If you wish to redistribute ARIA under different terms, contact 
Adept MobileRobots for information about a commercial version of ARIA at 
robots@mobilerobots.com or 
Adept MobileRobots, 10 Columbia Drive, Amherst, NH 03031; +1-603-881-7960
*/
/** @example imuExample.cpp  Example of gathering IMU data from Seekur

  Sample program that demonstrates how to get IMU data from an IMU on  
  a Seekur-class robot (from SeekurOS).  Only some Seekurs have IMUs.

  [Some Pioneers also have
  gyros that are integrated into the robot and normally automatically
  correct data or which can optionally return the data seperately
  (using different packets and different ARIA interface in ArAnalogGyro).]

  The data comes back in a custom packet with
  ID 0x9A.  It's requested with client command 26.

  Press escape to exit the program.

 */

#include "Aria.h"
#include <time.h>

bool imuPacketHandler(ArRobotPacket* packet) {
  if (packet->getID() != 0x9a) {
    //ArLog::log(ArLog::Normal, "ignoring packet with id: %d", packet->getID());
    return false;
  }
  int timeSince = packet->bufToByte();
  int numPerAxis = packet->bufToByte();

  ArLog::log(ArLog::Normal, "got IMUpac, %dms since readings took place, %d readings per axis", timeSince, numPerAxis);
  int i;
  int range;
  double x = 0.0;
  double y = 0.0;
  double z = 0.0;
  double multiplier = 0.0;
  double offset = 0.0;
  int numGyroAxes = packet->bufToByte();
  for (i=1; i <= numPerAxis; i++) {
    x = 0.0;
    y = 0.0;
    z = 0.0;
    range = packet->bufToByte();
    if (range == 3) {
      multiplier = 0.07326;
      offset = 0.0;
    }
    else if (range == 2) {
      multiplier = 0.03663;
      offset = 0.0;
    }
    else if (range == 1) {
      multiplier = 0.01832;
      offset = 0.0;
    }
    if (numGyroAxes >= 1) {
      x = (packet->bufToByte2() * multiplier) + offset;
    }
    if (numGyroAxes >= 2) {
      y = (packet->bufToByte2() * multiplier) + offset;
    }
    if (numGyroAxes >= 3) {
      z = (packet->bufToByte2() * multiplier) + offset;
    }
    ArLog::log(ArLog::Normal, "gyro:  reading %d of %d,       x: %10.4f deg/s,   y: %10.4f deg/s,   z: %10.4f deg/s", 
	       i, numPerAxis, x, y, z);
  }
  int numAccelAxes = packet->bufToByte();
  for (i=1; i <= numPerAxis; i++) {
    x = 0.0;
    y = 0.0;
    z = 0.0;
    multiplier = 0.002522 * 9806.65;
    offset = 0.0;
    if (numAccelAxes >= 1) {
      x = (packet->bufToByte2() * multiplier) + offset;
    }
    if (numAccelAxes >= 2) {
      y = (packet->bufToByte2() * multiplier) + offset;
    }
    if (numAccelAxes >= 3) {
      z = (packet->bufToByte2() * multiplier) + offset;
    }
    ArLog::log(ArLog::Normal, "accel: reading %d of %d,       x: %10.4f mg,      y: %10.4f mg,      z: %10.4f mg",
	       i, numPerAxis, x, y, z);
  }
  int numTemperatureAxes = packet->bufToByte();
  for (i=1; i <= numPerAxis; i++) {
    x = 0.0;
    y = 0.0;
    z = 0.0;
    multiplier = 0.1453;
    offset = 25.0;
    if (numTemperatureAxes >= 1) {
      x = (packet->bufToByte2() * multiplier) + offset;
    }
    if (numTemperatureAxes >= 2) {
      y = (packet->bufToByte2() * multiplier) + offset;
    }
    if (numTemperatureAxes >= 3) {
      z = (packet->bufToByte2() * multiplier) + offset;
    }
    ArLog::log(ArLog::Normal, "temperature: reading %d of %d, x: %10.4f degC,    y: %10.4f degC,    z: %10.4f degC",
	       i, numPerAxis, x, y, z);
  }
  ArLog::log(ArLog::Normal, "");
  ArLog::log(ArLog::Normal, "");

  return true;
}

int main(int argc, char **argv) {
  Aria::init();
  ArLog::init(ArLog::StdOut, ArLog::Normal, "", true);
  ArArgumentParser parser(&argc, argv);
  ArRobot *robot = new ArRobot;
  ArSimpleConnector simpleConnector(&parser);
  ArKeyHandler keyHandler;
  Aria::setKeyHandler(&keyHandler);
  robot->attachKeyHandler(&keyHandler);
  
  parser.loadDefaultArguments();
  if (!simpleConnector.parseArgs() || !parser.checkHelpAndWarnUnparsed(1)) {
    simpleConnector.logOptions();
    keyHandler.restore();
    Aria::exit(1);
  }
  if (!simpleConnector.connectRobot(robot)) {
    ArLog::log(ArLog::Terse, "Error connecting to robot.");
    Aria::shutdown();
    keyHandler.restore();
    Aria::exit(1);
  }
  ArLog::log(ArLog::Normal, "Connected to robot.");
  ArGlobalRetFunctor1<bool, ArRobotPacket *> myImuPacketHandler(&imuPacketHandler);
  robot->addPacketHandler(&myImuPacketHandler, ArListPos::FIRST);
  robot->runAsync(true);
  ArUtil::sleep(500);
  robot->lock();
  robot->comInt(26, 2);  // request IMU packets continuously
  ArLog::log(ArLog::Normal, "Requested IMU packets, waiting...");
  robot->unlock();
  robot->waitForRunExit();
  Aria::shutdown();
  exit(0);
}
			 
