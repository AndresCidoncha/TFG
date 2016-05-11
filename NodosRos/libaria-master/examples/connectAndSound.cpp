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
#include <iostream>
#include <vector>
#include "ariaTypedefs.h"
#include "ariaUtil.h"
#include "ArSoundsQueue.h"
#include "ArSoundPlayer.h"

/** @example simpleConnect.cpp example showing how to connect to the robot with ArRobotConnector
 *
 * One of the simplest ARIA programs possible:
 * Connects with ArRobotConnector, waits 3 seconds doing
 * nothing, then exits.
 *
 * This program will work either with the MobileSim simulator or on a real
 * robot's onboard computer.  (Or use -remoteHost to connect to a wireless
 * ethernet-serial bridge.)
 */

using namespace std;

void queueNowEmpty() {
  printf("The sound queue is now empty.\n");
}

void queueNowNonempty() {
  printf("The sound queue is now non-empty.\n");
}

bool no() {
  // just a false tautology
  return false;
}

int menuSonido(int argc, char **argv) {
  Aria::init();
  //ArLog::init(ArLog::StdErr, ArLog::Verbose);

  // Create the sound queue.
  ArSoundsQueue soundQueue;

  // Set WAV file callbacks
  soundQueue.setPlayWavFileCallback(ArSoundPlayer::getPlayWavFileCallback());
  soundQueue.setInterruptWavFileCallback(ArSoundPlayer::getStopPlayingCallback());

  // Notifications when the queue goes empty or non-empty.
  soundQueue.addQueueEmptyCallback(new ArGlobalFunctor(&queueNowEmpty));
  soundQueue.addQueueNonemptyCallback(new ArGlobalFunctor(&queueNowNonempty));

  // Run the sound queue in a new thread
  soundQueue.runAsync();

  // Get WAV file names from command line
  if(argc < 2)
  {
    cerr << "Usage: " << argv[0] << " <up to ten WAV sound file names...>\n";
    Aria::exit(-1);
  }
  std::vector<const char*> filenames;
  for(int i = 1; i < min(argc, 11); i++)
  {
    filenames.push_back(argv[i]);
  }

  // This functor can be used to cancel all sound playback until removed
  ArGlobalRetFunctor<bool> dontPlayItem(&no);

  while(Aria::getRunning())
  {
    std::cout << "Queue is " <<
      string(soundQueue.isPaused()?"paused":(soundQueue.isPlaying()?"playing":"ready"))
      << ", with " << soundQueue.getCurrentQueueSize() << " pending sounds." << std::endl
      << "Enter a command followed by the enter key:\n"
      << "\tp\trequest pause state (cumulative)\n"
      << "\tr\trequest resume state (cumulative)\n"
      << "\ti\tinterrupt current sound\n"
      << "\tc\tclear the queue\n"
      << "\tC\tclear priority < 4 from the queue.\n"
      << "\tn\tAdd " << filenames[0] << " to the queue, but with a condition callback to prevent it playing.\n"
      << "\tv\tAdjust volume -50%\n"
      << "\tV\tAdjust volume +50%\n"
      << "\to\tAdjust volume -100%\n"
      << "\tO\tAdjust volume +100%\n"
      << "\tl\tAdjust volume -200%\n"
      << "\tL\tAdjust volume +200%\n"
      << "\t-\tSet volume adjustment to normal level\n"
      ;
    for(size_t i = 0; i < filenames.size(); i++)
      std::cout << "\t" << i << "\tadd " << filenames[i] << " to the queue\n";
    std::cout << "\tq\tquit\n\n";

    int c = getchar();
    if(c == '\n')
      continue;
    switch(c)
    {
      case 'p': soundQueue.pause(); break;
      case 'r': soundQueue.resume(); break;
      case 'i': soundQueue.interrupt(); break;
      case 'q': soundQueue.stop(); ArUtil::sleep(100); Aria::exit(0);
      case 'c': soundQueue.clearQueue(); break;
      case 'C': soundQueue.removePendingItems(4); break;
      case 'n':
      {
        std::cout << "Adding \"" << filenames[0] << "\" but with a condition callback that will prevent it from playing...\n";
        ArSoundsQueue::Item item = soundQueue.createDefaultFileItem(filenames[0]);
        item.playbackConditionCallbacks.push_back(&dontPlayItem);
        soundQueue.addItem(item);
        break;
      }
      case 'v': ArSoundPlayer::setVolumePercent(-50.0); break;
      case 'V': ArSoundPlayer::setVolumePercent(50.0); break;
      case 'o': ArSoundPlayer::setVolumePercent(-100.0); break;
      case 'O': ArSoundPlayer::setVolumePercent(100.0); break;
      case 'l': ArSoundPlayer::setVolumePercent(-200.0); break;
      case 'L': ArSoundPlayer::setVolumePercent(200.0); break;
      case '-': ArSoundPlayer::setVolumePercent(0.0); break;
      default:
        if(filenames.size() > 0 && c >= '0' && c <= '9')
        {
          size_t i = c - '0';
          if(i < filenames.size())
          {
            std::cout << "Adding \"" << filenames[i] << "\" to the queue...\n";
            soundQueue.play(filenames[i]);
          }
        }
    }
  }
  std::cout << "ended.\n";
  return 0;
}

int main(int argc, char **argv)
{
  Aria::init();
  ArArgumentParser parser(&argc, argv);
  parser.loadDefaultArguments();
  ArRobot robot;

  // Connect to the robot, get some initial data from it such as type and name,
  // and then load parameter files for this robot.
  ArRobotConnector robotConnector(&parser, &robot);
  if(!robotConnector.connectRobot())
  {
    ArLog::log(ArLog::Terse, "simpleConnect: Could not connect to the robot.");
    if(parser.checkHelpAndWarnUnparsed())
    {
        // -help not given
        Aria::logOptions();
        Aria::exit(1);
    }
  }

  if (!Aria::parseArgs() || !parser.checkHelpAndWarnUnparsed())
  {
    Aria::logOptions();
    Aria::exit(1);
  }

  ArLog::log(ArLog::Normal, "simpleConnect: Connected.");

  robot.comInt(ArCommands::JOYINFO, 0);

  // Start the robot processing cycle running in the background.
  // True parameter means that if the connection is lost, then the 
  // run loop ends.
  robot.runAsync(true);

  // Print out some data from the SIP.  We must "lock" the ArRobot object
  // before calling its methods, and "unlock" when done, to prevent conflicts
  // with the background thread started by the call to robot.runAsync() above.
  // See the section on threading in the manual for more about this.
  robot.lock();
  ArLog::log(ArLog::Normal, "simpleConnect: Pose=(%.2f,%.2f,%.2f), Trans. Vel=%.2f, Battery=%.2fV",
    robot.getX(), robot.getY(), robot.getTh(), robot.getVel(), robot.getBatteryVoltage());
  robot.unlock();

  // Sleep for 3 seconds.
  ArLog::log(ArLog::Normal, "simpleConnect: Sleeping for 3 seconds...");
  ArUtil::sleep(3000);

  menuSonido(argc, argv);

  ArLog::log(ArLog::Normal, "simpleConnect: Ending robot thread...");
  robot.stopRunning();

  // wait for the thread to stop
  robot.waitForRunExit();

  // exit
  ArLog::log(ArLog::Normal, "simpleConnect: Exiting.");
  Aria::exit(0);
  return 0;
}
