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
#include "Aria.h"

/** @example robotConnectionCallbacks.cpp Demonstrates how callbacks can be registered with ArRobot to be
 *  called when a connection succeeds or fails.
*/

/*
  This class contains the methods called by the connection callback functors.
*/
class ConnHandler
{
public:
  // Constructor
  ConnHandler(ArRobot *robot);

  // Destructor, its just empty
  ~ConnHandler(void) {}
  
  // called if the connection was sucessfully made
  void connected(void);

  // called if the connection failed. stop the robot processing thread.
  void connectionFailed(void);

  // called when the connection is closed
  void disconnected(void);

  // called if the connection is lost due to an error
  void connectionLost(void);

protected:
  // keep a robot pointer
  ArRobot *myRobot;

  // the functor callbacks
  ArFunctorC<ConnHandler> myConnectedCB;
  ArFunctorC<ConnHandler> myConnFailCB;
  ArFunctorC<ConnHandler> myDisconnectedCB;
  ArFunctorC<ConnHandler> myConnLostCB;
};

/* ConnHandler constructor. Initialize functor objects, then
 * add them as connection handler callbacks with the robot object.
*/
ConnHandler::ConnHandler(ArRobot *robot) :
  myConnectedCB(this, &ConnHandler::connected),  
  myConnFailCB(this, &ConnHandler::connectionFailed),
  myDisconnectedCB(this, &ConnHandler::disconnected),
  myConnLostCB(this, &ConnHandler::connectionLost)

{
  // keep a robot pointer for use by the handler callback methods
  myRobot = robot;

  // add the callbacks to the robot
  myRobot->addConnectCB(&myConnectedCB, ArListPos::FIRST);
  myRobot->addFailedConnectCB(&myConnFailCB, ArListPos::FIRST);
  myRobot->addDisconnectNormallyCB(&myDisconnectedCB, ArListPos::FIRST);
  myRobot->addDisconnectOnErrorCB(&myDisconnectedCB, ArListPos::FIRST);
}

// just exit if the connection failed
void ConnHandler::connectionFailed(void)
{
  ArLog::log(ArLog::Normal, "ConnHandler: Connection failed. Exiting the program.");
  Aria::exit(1);
}

void ConnHandler::connected(void)
{
  ArLog::log(ArLog::Normal, "ConnHandler: Connected. Turning off sonar,");
  // turn off sonar, turn off amigobot sounds
  myRobot->comInt(ArCommands::SONAR, 0);
  myRobot->comInt(ArCommands::SOUNDTOG, 0);
}

// disconnected
void ConnHandler::disconnected(void)
{
  ArLog::log(ArLog::Normal, "ConnHandler: Connection closed. Exiting the program.");
  Aria::exit(0);
}

// lost connection due to errror, exit
void ConnHandler::connectionLost(void)
{
  ArLog::log(ArLog::Normal, "ConnHandler: Lost connection due to an error! Exiting the program!");
  Aria::exit(1);
}



int main(int argc, char **argv) 
{
  Aria::init();
  ArRobot robot;
  ArArgumentParser argParser(&argc, argv);
  ArSimpleConnector con(&argParser);
  if(!Aria::parseArgs() || !argParser.checkHelpAndWarnUnparsed())
  {
    Aria::logOptions();
    return 1;
  }

  // Create a connection handler object, defined above, then try to connect to the
  // robot.
  ConnHandler ch(&robot);
  con.connectRobot(&robot);
  robot.runAsync(true);

  // Sleep for 10 seconds, then request that ArRobot stop its thread.
  ArLog::log(ArLog::Normal, "Sleeping for 10 seconds...");
  ArUtil::sleep(10000);
  ArLog::log(ArLog::Normal, "...requesting that the robot thread exit, then shutting down ARIA and exiting.");
  robot.stopRunning();
  Aria::exit(0);
  return 0;
}

