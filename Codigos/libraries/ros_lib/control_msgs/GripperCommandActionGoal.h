#ifndef _ROS_control_msgs_GripperCommandActionGoal_h
#define _ROS_control_msgs_GripperCommandActionGoal_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"
#include "actionlib_msgs/GoalID.h"
#include "control_msgs/GripperCommandGoal.h"

namespace control_msgs
{

  class GripperCommandActionGoal : public ros::Msg
  {
    public:
      std_msgs::Header header;
      actionlib_msgs::GoalID goal_id;
      control_msgs::GripperCommandGoal goal;

    GripperCommandActionGoal():
      header(),
      goal_id(),
      goal()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += this->header.serialize(outbuffer + offset);
      offset += this->goal_id.serialize(outbuffer + offset);
      offset += this->goal.serialize(outbuffer + offset);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->header.deserialize(inbuffer + offset);
      offset += this->goal_id.deserialize(inbuffer + offset);
      offset += this->goal.deserialize(inbuffer + offset);
     return offset;
    }

    const char * getType(){ return "control_msgs/GripperCommandActionGoal"; };
    const char * getMD5(){ return "aa581f648a35ed681db2ec0bf7a82bea"; };

  };

}
#endif