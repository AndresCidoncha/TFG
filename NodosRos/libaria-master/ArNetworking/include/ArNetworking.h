#ifndef ARNETWORKING_H
#define ARNETWORKING_H

#include "ArServerBase.h"
#include "ArServerClient.h"
#include "ArServerCommands.h"
#include "ArClientBase.h"
#include "ArClientCommands.h"
#include "ArMapChanger.h"
#include "ArServerHandlerCamera.h"
#include "ArServerHandlerCameraCollection.h"
#include "ArServerHandlerCommMonitor.h"
#include "ArServerHandlerCommands.h"
#include "ArServerHandlerPopup.h"
#include "ArServerInfoDrawings.h"
#include "ArServerInfoRobot.h"
#include "ArServerInfoSensor.h"
#include "ArServerHandlerMap.h"
#include "ArServerMode.h"
#include "ArServerModeDrive.h"
#include "ArServerModeRatioDrive.h"
#include "ArServerModeStop.h"
#include "ArServerModeWander.h"
#include "ArServerHandlerConfig.h"
#include "ArClientHandlerConfig.h"
#include "ArHybridForwarderVideo.h"
#include "ArServerSimpleCommands.h"
#ifndef WIN32
#include "ArServerFileUtils.h"
#endif
#include "ArClientFileUtils.h"
#include "ArServerUserInfo.h"
#include "ArClientSimpleConnector.h"
#include "ArServerHandlerMapping.h"
#include "ArServerSimpleOpener.h"
#include "ArServerInfoStrings.h"
#include "ArClientArgUtils.h"
#include "ArServerHandlerPopup.h"
#include "ArCentralManager.h"
#include "ArCentralForwarder.h"
#include "ArClientSwitchManager.h"
#include "ArServerModeIdle.h"
#include "ArTempDirectoryHelper.h"

/**
   \mainpage ArNetworking
   
   MobileRobots <b>Advanced Robotics Networking Infrastructure (ArNetworking)</b>
   Developer's API Reference Manual<br>
   Copyright 2005 ActivMedia Robotics, LLC. All rights reserved.<br>
   Copyright 2006, 2007, 2008, 2009, 2010 MobileRobots Inc. All rights reserved.


   ArNetworking is an extensible networking protocol and infrastructure. It 
   is used to add networking services to a robot control program, integrated
   with ARIA and other MobileRobots software libraries.

   ArNetworking is used to set up a client-server architecture. A client 
   issues <i>requests</i> to a server to issue a single command the server, 
   or to start retrieving data from the server at a specified time interval.
   A request type is identified by a short string. 

   A client may be an operator's graphical interface, or a component requiring
   off-board resources (such as a multi-robot planning system, for example).   
   However, a program onboard the robot could also implement a client, and issue
   requests to an offboard server (e.g. to retrieve data from a central database).

   In a typical application, however, a program acting as the server runs on the robot's 
   computer, and uses ARIA (and possibly ARNL or SONARNL) to control the robot.  
   This creates a three-tier system: the robot itself is the first tier, and acts as a 
   "robot server" to a program on its onboard computer which is simultaneously a 
   "robot client" and an "ArNetworking server".  Offboard ArNetworking clients are the final
   tier.  This three-tier architecture allows a tight, reliable control loop between the robot
   control program on the onboard computer, and removes higher level user-interface clients
   accross the potentially unreliable network, protecting the more critical robot control program,
   and providing a more immediate and flexible user interface.

   ArNetworking can use both TCP and UDP.

   \image html ArNetworking_overview.png Abstract overview of ArNetworking in a typical client-server configuration with one client issuing requests to a server

   \section Servers Servers

   An ArNetworking server program is comprised of an ArServerBase object, to which
   callback functors are added for each request type you wish to handle (keyed
   by a request identifier string, though this string is internally converted
   into a more efficient integer value) using ArServerBase::addData().
   These callbacks are often encapsulated in "ServerHandler" or "ServerInfo" 
   classes. The ArNetworking library provides a variety of prewritten classes for 
   providing current state information about the robot, and for teleoperation
   control.    A callback functor for a request, when invoked, receives an ArNetPacket
   object containing the request's payload data, and a pointer
   to an object used to send a reply packet to the client. 
   ArServerBase creates a background thread to asyncronously accept client connections
   and receive clients' requests, while your main program thread executes simultaneously.
   This thread runs in a loop, and you may add general-purpose callback functors
   to be invoked each cycle of this loop using ArServerBase::addCycleCallback().

   To simplify server program setup, a helper class ArServerSimpleOpener is available.

   The default port number for an ArServerBase object to open is 7272, though
   an alternate port number may be used via a command line argument to the program,
   or specified in the ArServerBase constructor.

   \subsection ServerModes Server Modes

   ArNetworking also provides a system of server mode classes (see ArServerMode
   base class).   The server may only be in one mode at a time, each of which
   enables and disables ARIA ArAction objects.

   \subsection SimpleServerCommands Simple Server Commands

   ArNetworking includes the ArServerHandlerCommands class that provides a very 
   simple API for simple "command" requests: requests, perhaps with a few 
   parameters, that simply perform some action or change some state, and have 
   no data to reply with.    Callback functors added for requests via 
   an ArServerHandlerCommands object do not receive the request packet data, so they
   do not need to do any decoding of that data. Additionally, ArServerHandlerCommands
   keeps a list of requests that it manages, which is available to clients.
   ArNetworking includes some prewritten classes (ArServerSimpleCom*) which
   simply add new commands to an ArServerHandlerCommands object and encapsulate their
   request callbacks.

   For example, MobileEyes queries this list on 
   connection, and if some of these simple
   commands are available, it makes a menu in the toolbar called Custom Commands
   accessible.  This is a way to make special features of your custom robot server
   application controllable from MobileEyes.

   \subsection ARNLServerClasses Networking in ARNL and SONARNL

   The ARNL and SONARNL libraries contain some ServerHandler/ServerInfo classes 
   that implement useful server requests, as well as server Mode classes.

   \subsection PreMadeServices Standard Services Available in ArNetworking

   These are some of the pre-written services which are available in the ArNetworking
   library for you to use in your server programs.  To use these services, your 
   server program typically only needs to create and retain an instance of the class,
   though some require some additional setup.

   <ul>
    <li>ArServerInfoRobot - Supplies clients with basic robot state information (current position and velocity, active server mode and status, battery voltage)</li>
    <li>ArServerInfoSensor - Supplies clients with current sensor readings (Sonar or Laser)</li>
    <li>ArServerHandlerMap - Supplies clients with data from an ArMap</li>
    <li>ArServerInfoDrawings - Supplies clients with a set of graphical figures to be displayed with the map (e.g. point sets, lines, circles)</li>
    <li>ArServerHandlerCamera - Allows clients to control a pan-tilt camera and provides information about its current position</li>
    <li>ArServerInfoStrings - A set of general purpose key,value string pairs obtained from a global table (See Aria::getInfoGroup())</li>
    <li>ArServerHandlerMapping - Allows clients to trigger collection of laser scan data for use in making a map later (with the Mapper3 application)</li>
    <li>ArServerFileToClient, ArServerFileFromClient, ArServerFileLister - Allows clients to upload and download files from the server filesystem</li>
   </ul>
   

   \section Clients Clients

   A client program is comprised of an ArClientBase object which must be connected
   to a server via a network socket (either TCP or the default, UDP). The server is 
   identified by network hostname and port number, which may be given
   through ArClientBase API or in the program command line arguments. Requests
   are made to the server via ArClientBase::request() or ArClientBase::requestOnce().
   Callback functors may be added to the ArClientBase object to receive reply packets.
   ArClientBase creates a background thread to asyncronously communicate with the server.

   A client may query a server to discover if a data request type is available using
   ArClientBase::dataExists().

   To simplify client program setup and connection to a server, a helper class
   ArClientSimpleConnector is available.


   \section Auth Authenticated Connections 

   A simple scheme for authenticated connections is available in ArNetworking.
   When connecting to a server, a client may supply a username and password.
   If the server has loaded a <i>user info file</i>, it may reject a connection
   with an incorrect password.  Data requests may be categorized into 
   <i>command groups</i>, and which command groups are allowed to which users may
   be specified in the user info file.  
   
   The login process is not impossible to defeat with some concerted effort and expertise,
   but is sufficient to prevent basic attempts at unauthorized use of a server, and 
   to present only relevant features for different users' purposes.
  
   See ArServerBase for more discussion.

**/

#endif // ARNETWORKING
