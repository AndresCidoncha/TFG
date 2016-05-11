
#include "Aria.h"
#include "ArExport.h"

#include "ArClientArgUtils.h"

#include <ArConfigArg.h>

#include "ArNetPacket.h"

//#define ARDEBUG_CLIENTARGUTILS

#if (defined(_DEBUG) && defined(ARDEBUG_CLIENTARGUTILS))
#define IFDEBUG(code) {code;}
#else
#define IFDEBUG(code)
#endif 

AREXPORT ArClientArg::ArClientArg(bool isDisplayHintParsed,
                                  ArPriority::Priority lastPriority,
                                  int version) :
  myIsDisplayHintParsed(isDisplayHintParsed),
  myLastPriority(lastPriority),
  myVersion(version),
  myBuffer(),
  myDisplayBuffer()
{}

AREXPORT ArClientArg::~ArClientArg()
{}

AREXPORT bool ArClientArg::isSendableParamType(const ArConfigArg &arg)
{
  switch (arg.getType()) {
  case ArConfigArg::INT:
  case ArConfigArg::DOUBLE:
  case ArConfigArg::BOOL:
  case ArConfigArg::LIST:
  case ArConfigArg::STRING:
  case ArConfigArg::SEPARATOR:
    return true;

  default:
    return false;
  }
}


AREXPORT bool ArClientArg::createArg(ArNetPacket *packet, 
							                       ArConfigArg &argOut)
{
	if (packet == NULL) {
    ArLog::log(ArLog::Verbose, "ArClientArg::createArg() cannot unpack NULL packet");
		return false;
	}

	bool isSuccess = true;

	char name[32000];
	char description[32000];
  myDisplayBuffer[0] = '\0';

	packet->bufToStr(name, sizeof(name));
	packet->bufToStr(description, sizeof(description));


	char priorityVal = packet->bufToByte();
 
	ArPriority::Priority priority = myLastPriority;
  if ((priorityVal >= 0) && (priorityVal <= myLastPriority)) {
    priority = (ArPriority::Priority) priorityVal;
  }

	char argType = packet->bufToByte();

 switch (argType) {
	
 case 'B':
 case 'b': // Lower case indicates display information contained in packet...
	  {
      if ((argType == 'B') || (myIsDisplayHintParsed)) {

		    bool boolVal = false;
		    if (packet->bufToByte()) {
			    boolVal = true;
		    }
		    //packet->bufToStr(myDisplayBuffer, BUFFER_LENGTH);
		    argOut = ArConfigArg(name, boolVal, description);
      }
      else {
        isSuccess = false;
      }
	  }
    break;

  case 'I':
  case 'i':  // Lower case indicates display information contained in packet...
	  {
      if ((argType == 'I') || (myIsDisplayHintParsed)) {
  
		    int intVal = packet->bufToByte4();
		    int intMin = packet->bufToByte4();
		    int intMax = packet->bufToByte4();

		    argOut = ArConfigArg(name, intVal, description, intMin, intMax);
      }
      else {
        isSuccess = false;
      }
  
	  }
    break;

  case 'D':
	case 'd': // Lower case indicates display information contained in packet...
  {
    if ((argType == 'D') || (myIsDisplayHintParsed)) {
 		  double doubleVal = packet->bufToDouble();
		  double doubleMin = packet->bufToDouble();
		  double doubleMax = packet->bufToDouble();
      
      if (myVersion >= 2) {
        int precision = packet->bufToByte4();
        argOut = ArConfigArg(name, doubleVal, description, doubleMin, doubleMax, precision);
      }
      else {
		    argOut = ArConfigArg(name, doubleVal, description, doubleMin, doubleMax, -1);
      }
	  }
    else {
      isSuccess = false;
    }
  }
  break;

  case 'S':
  case 's': // Lower case indicates display information contained in packet...
	  {
      if ((argType == 'S') || (myIsDisplayHintParsed)) {

		    packet->bufToStr(myBuffer, BUFFER_LENGTH);
				
        //packet->bufToStr(myDisplayBuffer, BUFFER_LENGTH);
  	    argOut = ArConfigArg(name, myBuffer, description, 0);
      }
      else {
        isSuccess = false;
      }
	  }  
    break;

  case 'L':
  case 'l': // Lower case indicates display information contained in packet...
	  {

      if ((argType == 'L') || (myIsDisplayHintParsed)) {
		 
        int childCount = packet->bufToByte4();

        ArConfigArg listArg(ArConfigArg::LIST, name, description);
        ArConfigArg childArg;

        for (int i = 0; ((i < childCount) && (isSuccess)); i++) {
          isSuccess = createArg(packet, 
							                  childArg);
          if (isSuccess) {
            listArg.addArg(childArg);
          }
        }

        if (isSuccess) {
          argOut = listArg;
        }
      }
      else {
        isSuccess = false;
      }
	  }  
    break;

  case '.':
    {
       //if (myIsDisplayHintParsed) {
			 //packet->bufToStr(myDisplayBuffer, BUFFER_LENGTH);
       //}
       argOut = ArConfigArg(ArConfigArg::SEPARATOR);
    }
    break;

  default:

		isSuccess = false;
    ArLog::log(ArLog::Terse, "ArClientArg::createArg() unsupported param type %c",
               argType);
	}

  argOut.setConfigPriority(priority);
  if (myIsDisplayHintParsed) {

    if (isSuccess) {
		  packet->bufToStr(myDisplayBuffer, BUFFER_LENGTH);
    }

    IFDEBUG(
      if (strlen(myDisplayBuffer) > 0) {
        ArLog::log(ArLog::Verbose, "ArClientArg::createArg() arg %s has displayHint = %s",
                  argOut.getName(), myDisplayBuffer);
      }
    );

    argOut.setDisplayHint(myDisplayBuffer);
  }

  if (myVersion >= 2) {
    if (isSuccess) {
		  packet->bufToStr(myExtraBuffer, BUFFER_LENGTH);
    }

    IFDEBUG(
      if (strlen(myExtraBuffer) > 0) {
        ArLog::log(ArLog::Verbose, "ArClientArg::createArg() arg %s has extra explanation = %s",
                  argOut.getName(), myExtraBuffer);
      }
    );

    argOut.setExtraExplanation(myExtraBuffer);
  } // end if 

  if (myVersion >= 2) {
    char restartVal = packet->bufToByte();
    ArConfigArg::RestartLevel restart = ArConfigArg::NO_RESTART;
    if ((restartVal >= 0) && (restartVal <= ArConfigArg::LAST_RESTART_LEVEL)) {
      restart = (ArConfigArg::RestartLevel) restartVal;
      argOut.setRestartLevel(restart);
    }  
  }

	return isSuccess;

} // end method createArg


AREXPORT bool ArClientArg::createPacket(const ArConfigArg &arg,
                                        ArNetPacket *packet)
{
  if (packet == NULL) {
    ArLog::log(ArLog::Verbose, 
               "ArClientArg::createPacket() cannot create NULL packet");
    return false;
  }

	bool isSuccess = true;

	packet->strToBuf(arg.getName());
	packet->strToBuf(arg.getDescription());
  
  ArPriority::Priority priority = arg.getConfigPriority(); 
  if (priority > myLastPriority) {
    priority = myLastPriority;
  }
 
	packet->byteToBuf(priority);

	char argType = '\0';
  
  switch (arg.getType()) {
  case ArConfigArg::BOOL:

    argType = (myIsDisplayHintParsed ? 'b' : 'B');

    packet->byteToBuf(argType);
    packet->byteToBuf(arg.getBool());
    break;
  
  case ArConfigArg::INT:

    argType = (myIsDisplayHintParsed ? 'i' : 'I');
    packet->byteToBuf(argType);

		packet->byte4ToBuf(arg.getInt());
		packet->byte4ToBuf(arg.getMinInt());
		packet->byte4ToBuf(arg.getMaxInt());

    break;

  case ArConfigArg::DOUBLE:

    argType = (myIsDisplayHintParsed ? 'd' : 'D');
    packet->byteToBuf(argType);

		packet->doubleToBuf(arg.getDouble());
		packet->doubleToBuf(arg.getMinDouble());
		packet->doubleToBuf(arg.getMaxDouble());
    if (myVersion >= 2) {
      packet->byte4ToBuf(arg.getDoublePrecision());
    }
    break;

  case ArConfigArg::STRING:

    argType = (myIsDisplayHintParsed ? 's' : 'S');
    packet->byteToBuf(argType);

		packet->strToBuf(arg.getString());

    break;

 case ArConfigArg::LIST:
   {
    argType = (myIsDisplayHintParsed ? 'l' : 'L');
    packet->byteToBuf(argType);

		
    
    std::list<ArConfigArg> childList = arg.getArgs();
   
    int sendableCount = 0;

    for (std::list<ArConfigArg>::const_iterator iter1 = childList.begin();
         iter1 != childList.end();
         iter1++) {
      if (isSendableParamType(*iter1)) {
        sendableCount++;
      }
    }

    packet->byte4ToBuf(sendableCount);

    for (std::list<ArConfigArg>::const_iterator iter2 = childList.begin();
         ((iter2 != childList.end()) && (isSuccess));
         iter2++) {
      if (isSendableParamType(*iter2)) {
        if (!createPacket(*iter2, packet)) {
          isSuccess = false;
        }
      }
    } // end for each child
   }
   break;

 case ArConfigArg::SEPARATOR:

    argType = '.';
    packet->byteToBuf(argType);
    break;

  default:

		isSuccess = false;
    ArLog::log(ArLog::Terse, 
               "ArClientArg::createPacket() unsupported param type %s", 
               ArConfigArg::toString(arg.getType()));

  } // end switch type

  if (isSuccess && myIsDisplayHintParsed) {
    packet->strToBuf(arg.getDisplayHint());
  }
  if (isSuccess && (myVersion >= 2)) {
    packet->strToBuf(arg.getExtraExplanation());
  }
  if (isSuccess && (myVersion >= 2)) {
    packet->byteToBuf(arg.getRestartLevel());
  }

	return isSuccess;

} // end method createPacket



AREXPORT bool ArClientArg::bufToArgValue(ArNetPacket *packet,
                                        ArConfigArg &arg)
{
  if (packet == NULL) {
    return false;
  }

  bool isSuccess = true;

  switch (arg.getType()) {
  case ArConfigArg::BOOL:
    {
		bool boolVal = false;
		if (packet->bufToByte()) {
			boolVal = true;
		}
    isSuccess = arg.setBool(boolVal);
    }
    break;
  
  case ArConfigArg::INT:
    {
		int intVal = packet->bufToByte4();
    isSuccess = arg.setInt(intVal);
    }
    break;

  case ArConfigArg::DOUBLE:
    {
    double doubleVal = packet->bufToDouble();
    isSuccess = arg.setDouble(doubleVal);
    }
    break;

  case ArConfigArg::STRING:
    {
		packet->bufToStr(myBuffer, BUFFER_LENGTH);
    isSuccess = arg.setString(myBuffer);
    }
    break;

  case ArConfigArg::LIST:
    {
		int childCount = packet->bufToByte4();
    
    if (childCount > 0) {

      int copiedCount = 0;

      for (size_t c = 0; ((isSuccess) && (c < arg.getArgCount())); c++) {

        ArConfigArg *childArg = arg.getArg(c);
        if (childArg == NULL) {
          isSuccess = false;
          break;
        }
        if (!isSendableParamType(*childArg)) {
          continue;
        }
        isSuccess = bufToArgValue(packet,
                                  *childArg);
        copiedCount++;

      } // end for each arg

      if (copiedCount != childCount) {
        ArLog::log(ArLog::Normal,
                   "Error reading list values, received %i children, copied %i",
                   childCount, copiedCount);
      }

    } // end if children

    }
    break;
  case ArConfigArg::SEPARATOR:
    // There is no value...
    break;

  default:

		isSuccess = false;
    ArLog::log(ArLog::Terse, "ArClientArg::createPacket() unsupported param type %i", arg.getType());

  } // end switch type

  return isSuccess;

} // end method bufToArgValue



AREXPORT bool ArClientArg::argValueToBuf(const ArConfigArg &arg,
                                         ArNetPacket *packet)
{
  if (packet == NULL) {
    return false;
  }

  bool isSuccess = true;

  switch (arg.getType()) {
  case ArConfigArg::BOOL:

    packet->byteToBuf(arg.getBool());
    break;
  
  case ArConfigArg::INT:

		packet->byte4ToBuf(arg.getInt());
    break;

  case ArConfigArg::DOUBLE:

    packet->doubleToBuf(arg.getDouble());
    break;

  case ArConfigArg::STRING:

		packet->strToBuf(arg.getString());
    break;

  case ArConfigArg::LIST:
    {
/****
      packet->byte4ToBuf(arg.getArgCount());
      
      std::list<ArConfigArg> childList = arg.getArgs();
      for (std::list<ArConfigArg>::const_iterator iter = childList.begin();
           ((isSuccess) && (iter != childList.end()));
           iter++) {
        isSuccess = argValueToBuf(*iter,
                                   packet);
      }
*****/ 

      std::list<ArConfigArg> childList = arg.getArgs();
     
      int sendableCount = 0;

      for (std::list<ArConfigArg>::const_iterator iter1 = childList.begin();
           iter1 != childList.end();
           iter1++) {
        if (isSendableParamType(*iter1)) {
          sendableCount++;
        }
      }

      packet->byte4ToBuf(sendableCount);

      for (std::list<ArConfigArg>::const_iterator iter2 = childList.begin();
           ((iter2 != childList.end()) && (isSuccess));
           iter2++) {
        if (isSendableParamType(*iter2)) {
          isSuccess = argValueToBuf(*iter2,
                                     packet);
  //        if (!createPacket(*iter2, packet)) {
  //          isSuccess = false;
  //        }
        }
      } // end for each child
    }
    break;

  case ArConfigArg::SEPARATOR:
    // There is no value...
    break;

  default:

		isSuccess = false;
    ArLog::log(ArLog::Terse, "ArClientArg::createPacket() unsupported param type %i", arg.getType());

  } // end switch type

  return isSuccess;

} // end method argValueToBuf


AREXPORT bool ArClientArg::argTextToBuf(const ArConfigArg &arg,
                                        ArNetPacket *packet)
{
  if (packet == NULL) {
    return false;
  }

  bool isSuccess = true;

  switch (arg.getType()) {
  case ArConfigArg::INT:
    snprintf(myBuffer, BUFFER_LENGTH, "%d", arg.getInt());
    break;
  case ArConfigArg::DOUBLE:
    snprintf(myBuffer, BUFFER_LENGTH, "%g", arg.getDouble());
    break;
  case ArConfigArg::BOOL:
    snprintf(myBuffer, BUFFER_LENGTH, "%s", 
             ArUtil::convertBool(arg.getBool()));
    break;

  case ArConfigArg::STRING:
    snprintf(myBuffer, BUFFER_LENGTH, "%s", arg.getString());
    break;

  case ArConfigArg::LIST:
    {

        ArLog::log(ArLog::Terse,
                   "UNSUPPORTED METHOD ArClientArg::argTextToBuf (LIST)");

        std::list<ArConfigArg> childList = arg.getArgs();

        for (std::list<ArConfigArg>::const_iterator iter2 = childList.begin();
             ((iter2 != childList.end()) && (isSuccess));
             iter2++) {
          if (isSendableParamType(*iter2)) {
 
            packet->strToBuf((*iter2).getName());

            isSuccess = ArClientArg::addArgTextToPacket(*iter2,
                                                         packet);
          }
        } // end for each child

     
        int sendableCount = 0;

 //       for (std::list<ArConfigArg>::const_iterator iter1 = childList.begin();
//           iter1 != childList.end();
//           iter1++) {
//        if (isSendableParamType(*iter1)) {
//          sendableCount++;
//        }
//      }

//	    snprintf(myBuffer, BUFFER_LENGTH, "%d", sendableCount);   

    }
    break;
  case ArConfigArg::SEPARATOR:
    break;
  default:
    isSuccess = false;
    break;
  } // end switch type

  if (isSuccess) {
    packet->strToBuf(myBuffer);
  }
  return isSuccess;

} // end method argTextToBuf

AREXPORT bool ArClientArg::addArgTextToPacket(const ArConfigArg &arg,
                                              ArNetPacket *packet)
{
  if (packet == NULL) {
    return false;
  }

  bool isSuccess = true;

  switch (arg.getType()) {
  case ArConfigArg::INT:
    packet->strToBuf(arg.getName());
    snprintf(myBuffer, BUFFER_LENGTH, "%d", arg.getInt());
    packet->strToBuf(myBuffer);
    break;
  case ArConfigArg::DOUBLE:
    packet->strToBuf(arg.getName());
    snprintf(myBuffer, BUFFER_LENGTH, "%g", arg.getDouble());
    packet->strToBuf(myBuffer);
    break;
  case ArConfigArg::BOOL:
    packet->strToBuf(arg.getName());
    snprintf(myBuffer, BUFFER_LENGTH, "%s", 
             ArUtil::convertBool(arg.getBool()));
    packet->strToBuf(myBuffer);
    break;

  case ArConfigArg::STRING:
    packet->strToBuf(arg.getName());
    snprintf(myBuffer, BUFFER_LENGTH, "%s", arg.getString());
    packet->strToBuf(myBuffer);
    break;

  case ArConfigArg::LIST:
    {
 /*     snprintf(myBuffer, BUFFER_LENGTH, "%s %s", 
               ArConfigArg::LIST_BEGIN_TAG,
               arg.getName());*/

      packet->strToBuf(ArConfigArg::LIST_BEGIN_TAG);
      packet->strToBuf(arg.getName());  
      
      std::list<ArConfigArg> childList = arg.getArgs();

      for (std::list<ArConfigArg>::const_iterator iter2 = childList.begin();
           ((iter2 != childList.end()) && (isSuccess));
           iter2++) {
        if (isSendableParamType(*iter2)) {

          // packet->strToBuf((*iter2).getName());

          isSuccess = ArClientArg::addArgTextToPacket(*iter2,
                                                      packet);
        }
      } // end for each child

      //snprintf(myBuffer, BUFFER_LENGTH, "%s %s", 
      //         ArConfigArg::LIST_END_TAG,
      //         arg.getName());
      //packet->strToBuf(myBuffer);

      packet->strToBuf(ArConfigArg::LIST_END_TAG);
      packet->strToBuf(arg.getName());  
 
    }
    break;
  case ArConfigArg::SEPARATOR:
    break;
  default:
    isSuccess = false;
    break;
  } // end switch type

  return isSuccess;

} // end method addArgTextToPacket

