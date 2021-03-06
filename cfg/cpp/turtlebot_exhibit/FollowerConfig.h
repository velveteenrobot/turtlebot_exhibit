//#line 2 "/opt/ros/electric/stacks/driver_common/dynamic_reconfigure/templates/ConfigType.h"
// *********************************************************
// 
// File autogenerated for the turtlebot_exhibit package 
// by the dynamic_reconfigure package.
// Please do not edit.
// 
// ********************************************************/

/***********************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2008, Willow Garage, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the Willow Garage nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 ***********************************************************/

// Author: Blaise Gassend


#ifndef __turtlebot_exhibit__FOLLOWERCONFIG_H__
#define __turtlebot_exhibit__FOLLOWERCONFIG_H__

#include <dynamic_reconfigure/config_tools.h>
#include <limits>
#include <ros/node_handle.h>
#include <dynamic_reconfigure/ConfigDescription.h>
#include <dynamic_reconfigure/ParamDescription.h>
#include <dynamic_reconfigure/config_init_mutex.h>

namespace turtlebot_exhibit
{
  class FollowerConfigStatics;
  
  class FollowerConfig
  {
  public:
    class AbstractParamDescription : public dynamic_reconfigure::ParamDescription
    {
    public:
      AbstractParamDescription(std::string n, std::string t, uint32_t l, 
          std::string d, std::string e)
      {
        name = n;
        type = t;
        level = l;
        description = d;
        edit_method = e;
      }
      
      virtual void clamp(FollowerConfig &config, const FollowerConfig &max, const FollowerConfig &min) const = 0;
      virtual void calcLevel(uint32_t &level, const FollowerConfig &config1, const FollowerConfig &config2) const = 0;
      virtual void fromServer(const ros::NodeHandle &nh, FollowerConfig &config) const = 0;
      virtual void toServer(const ros::NodeHandle &nh, const FollowerConfig &config) const = 0;
      virtual bool fromMessage(const dynamic_reconfigure::Config &msg, FollowerConfig &config) const = 0;
      virtual void toMessage(dynamic_reconfigure::Config &msg, const FollowerConfig &config) const = 0;
    };

    typedef boost::shared_ptr<AbstractParamDescription> AbstractParamDescriptionPtr;
    typedef boost::shared_ptr<const AbstractParamDescription> AbstractParamDescriptionConstPtr;
    
    template <class T>
    class ParamDescription : public AbstractParamDescription
    {
    public:
      ParamDescription(std::string name, std::string type, uint32_t level, 
          std::string description, std::string edit_method, T FollowerConfig::* f) :
        AbstractParamDescription(name, type, level, description, edit_method),
        field(f)
      {}

      T (FollowerConfig::* field);

      virtual void clamp(FollowerConfig &config, const FollowerConfig &max, const FollowerConfig &min) const
      {
        if (config.*field > max.*field)
          config.*field = max.*field;
        
        if (config.*field < min.*field)
          config.*field = min.*field;
      }

      virtual void calcLevel(uint32_t &comb_level, const FollowerConfig &config1, const FollowerConfig &config2) const
      {
        if (config1.*field != config2.*field)
          comb_level |= level;
      }

      virtual void fromServer(const ros::NodeHandle &nh, FollowerConfig &config) const
      {
        nh.getParam(name, config.*field);
      }

      virtual void toServer(const ros::NodeHandle &nh, const FollowerConfig &config) const
      {
        nh.setParam(name, config.*field);
      }

      virtual bool fromMessage(const dynamic_reconfigure::Config &msg, FollowerConfig &config) const
      {
        return dynamic_reconfigure::ConfigTools::getParameter(msg, name, config.*field);
      }

      virtual void toMessage(dynamic_reconfigure::Config &msg, const FollowerConfig &config) const
      {
        dynamic_reconfigure::ConfigTools::appendParameter(msg, name, config.*field);
      }
    };

//#line 42 "../cfg/Follower.cfg"
      double min_x;
//#line 43 "../cfg/Follower.cfg"
      double max_x;
//#line 44 "../cfg/Follower.cfg"
      double min_y;
//#line 45 "../cfg/Follower.cfg"
      double max_y;
//#line 46 "../cfg/Follower.cfg"
      double max_z;
//#line 47 "../cfg/Follower.cfg"
      double goal_z;
//#line 48 "../cfg/Follower.cfg"
      double x_scale;
//#line 49 "../cfg/Follower.cfg"
      double z_scale;
//#line 138 "/opt/ros/electric/stacks/driver_common/dynamic_reconfigure/templates/ConfigType.h"

    bool __fromMessage__(dynamic_reconfigure::Config &msg)
    {
      const std::vector<AbstractParamDescriptionConstPtr> &__param_descriptions__ = __getParamDescriptions__();
      int count = 0;
      for (std::vector<AbstractParamDescriptionConstPtr>::const_iterator i = __param_descriptions__.begin(); i != __param_descriptions__.end(); i++)
        if ((*i)->fromMessage(msg, *this))
          count++;
      if (count != dynamic_reconfigure::ConfigTools::size(msg))
      {
        ROS_ERROR("FollowerConfig::__fromMessage__ called with an unexpected parameter.");
        ROS_ERROR("Booleans:");
        for (unsigned int i = 0; i < msg.bools.size(); i++)
          ROS_ERROR("  %s", msg.bools[i].name.c_str());
        ROS_ERROR("Integers:");
        for (unsigned int i = 0; i < msg.ints.size(); i++)
          ROS_ERROR("  %s", msg.ints[i].name.c_str());
        ROS_ERROR("Doubles:");
        for (unsigned int i = 0; i < msg.doubles.size(); i++)
          ROS_ERROR("  %s", msg.doubles[i].name.c_str());
        ROS_ERROR("Strings:");
        for (unsigned int i = 0; i < msg.strs.size(); i++)
          ROS_ERROR("  %s", msg.strs[i].name.c_str());
        // @todo Check that there are no duplicates. Make this error more
        // explicit.
        return false;
      }
      return true;
    }

    // This version of __toMessage__ is used during initialization of
    // statics when __getParamDescriptions__ can't be called yet.
    void __toMessage__(dynamic_reconfigure::Config &msg, const std::vector<AbstractParamDescriptionConstPtr> &__param_descriptions__) const
    {
      dynamic_reconfigure::ConfigTools::clear(msg);
      for (std::vector<AbstractParamDescriptionConstPtr>::const_iterator i = __param_descriptions__.begin(); i != __param_descriptions__.end(); i++)
        (*i)->toMessage(msg, *this);
    }
    
    void __toMessage__(dynamic_reconfigure::Config &msg) const
    {
      const std::vector<AbstractParamDescriptionConstPtr> &__param_descriptions__ = __getParamDescriptions__();
      __toMessage__(msg, __param_descriptions__);
    }
    
    void __toServer__(const ros::NodeHandle &nh) const
    {
      const std::vector<AbstractParamDescriptionConstPtr> &__param_descriptions__ = __getParamDescriptions__();
      for (std::vector<AbstractParamDescriptionConstPtr>::const_iterator i = __param_descriptions__.begin(); i != __param_descriptions__.end(); i++)
        (*i)->toServer(nh, *this);
    }

    void __fromServer__(const ros::NodeHandle &nh)
    {
      const std::vector<AbstractParamDescriptionConstPtr> &__param_descriptions__ = __getParamDescriptions__();
      for (std::vector<AbstractParamDescriptionConstPtr>::const_iterator i = __param_descriptions__.begin(); i != __param_descriptions__.end(); i++)
        (*i)->fromServer(nh, *this);
    }

    void __clamp__()
    {
      const std::vector<AbstractParamDescriptionConstPtr> &__param_descriptions__ = __getParamDescriptions__();
      const FollowerConfig &__max__ = __getMax__();
      const FollowerConfig &__min__ = __getMin__();
      for (std::vector<AbstractParamDescriptionConstPtr>::const_iterator i = __param_descriptions__.begin(); i != __param_descriptions__.end(); i++)
        (*i)->clamp(*this, __max__, __min__);
    }

    uint32_t __level__(const FollowerConfig &config) const
    {
      const std::vector<AbstractParamDescriptionConstPtr> &__param_descriptions__ = __getParamDescriptions__();
      uint32_t level = 0;
      for (std::vector<AbstractParamDescriptionConstPtr>::const_iterator i = __param_descriptions__.begin(); i != __param_descriptions__.end(); i++)
        (*i)->calcLevel(level, config, *this);
      return level;
    }
    
    static const dynamic_reconfigure::ConfigDescription &__getDescriptionMessage__();
    static const FollowerConfig &__getDefault__();
    static const FollowerConfig &__getMax__();
    static const FollowerConfig &__getMin__();
    static const std::vector<AbstractParamDescriptionConstPtr> &__getParamDescriptions__();
    
  private:
    static const FollowerConfigStatics *__get_statics__();
  };
  
  template <> // Max and min are ignored for strings.
  inline void FollowerConfig::ParamDescription<std::string>::clamp(FollowerConfig &config, const FollowerConfig &max, const FollowerConfig &min) const
  {
    return;
  }

  class FollowerConfigStatics
  {
    friend class FollowerConfig;
    
    FollowerConfigStatics()
    {
//#line 42 "../cfg/Follower.cfg"
      __min__.min_x = -3.0;
//#line 42 "../cfg/Follower.cfg"
      __max__.min_x = 3.0;
//#line 42 "../cfg/Follower.cfg"
      __default__.min_x = -0.2;
//#line 42 "../cfg/Follower.cfg"
      __param_descriptions__.push_back(FollowerConfig::AbstractParamDescriptionConstPtr(new FollowerConfig::ParamDescription<double>("min_x", "double", 0, "The minimum x position of the points in the box.", "", &FollowerConfig::min_x)));
//#line 43 "../cfg/Follower.cfg"
      __min__.max_x = -3.0;
//#line 43 "../cfg/Follower.cfg"
      __max__.max_x = 3.0;
//#line 43 "../cfg/Follower.cfg"
      __default__.max_x = 0.2;
//#line 43 "../cfg/Follower.cfg"
      __param_descriptions__.push_back(FollowerConfig::AbstractParamDescriptionConstPtr(new FollowerConfig::ParamDescription<double>("max_x", "double", 0, "The maximum x position of the points in the box.", "", &FollowerConfig::max_x)));
//#line 44 "../cfg/Follower.cfg"
      __min__.min_y = -1.0;
//#line 44 "../cfg/Follower.cfg"
      __max__.min_y = 3.0;
//#line 44 "../cfg/Follower.cfg"
      __default__.min_y = 0.1;
//#line 44 "../cfg/Follower.cfg"
      __param_descriptions__.push_back(FollowerConfig::AbstractParamDescriptionConstPtr(new FollowerConfig::ParamDescription<double>("min_y", "double", 0, "The minimum y position of the points in the box.", "", &FollowerConfig::min_y)));
//#line 45 "../cfg/Follower.cfg"
      __min__.max_y = -1.0;
//#line 45 "../cfg/Follower.cfg"
      __max__.max_y = 3.0;
//#line 45 "../cfg/Follower.cfg"
      __default__.max_y = 0.5;
//#line 45 "../cfg/Follower.cfg"
      __param_descriptions__.push_back(FollowerConfig::AbstractParamDescriptionConstPtr(new FollowerConfig::ParamDescription<double>("max_y", "double", 0, "The maximum y position of the points in the box.", "", &FollowerConfig::max_y)));
//#line 46 "../cfg/Follower.cfg"
      __min__.max_z = 0.0;
//#line 46 "../cfg/Follower.cfg"
      __max__.max_z = 3.0;
//#line 46 "../cfg/Follower.cfg"
      __default__.max_z = 0.8;
//#line 46 "../cfg/Follower.cfg"
      __param_descriptions__.push_back(FollowerConfig::AbstractParamDescriptionConstPtr(new FollowerConfig::ParamDescription<double>("max_z", "double", 0, "The maximum z position of the points in the box.", "", &FollowerConfig::max_z)));
//#line 47 "../cfg/Follower.cfg"
      __min__.goal_z = 0.0;
//#line 47 "../cfg/Follower.cfg"
      __max__.goal_z = 3.0;
//#line 47 "../cfg/Follower.cfg"
      __default__.goal_z = 0.6;
//#line 47 "../cfg/Follower.cfg"
      __param_descriptions__.push_back(FollowerConfig::AbstractParamDescriptionConstPtr(new FollowerConfig::ParamDescription<double>("goal_z", "double", 0, "The distance away from the robot to hold the centroid.", "", &FollowerConfig::goal_z)));
//#line 48 "../cfg/Follower.cfg"
      __min__.x_scale = 0.0;
//#line 48 "../cfg/Follower.cfg"
      __max__.x_scale = 3.0;
//#line 48 "../cfg/Follower.cfg"
      __default__.x_scale = 1.0;
//#line 48 "../cfg/Follower.cfg"
      __param_descriptions__.push_back(FollowerConfig::AbstractParamDescriptionConstPtr(new FollowerConfig::ParamDescription<double>("x_scale", "double", 0, "The scaling factor for translational robot speed.", "", &FollowerConfig::x_scale)));
//#line 49 "../cfg/Follower.cfg"
      __min__.z_scale = 0.0;
//#line 49 "../cfg/Follower.cfg"
      __max__.z_scale = 10.0;
//#line 49 "../cfg/Follower.cfg"
      __default__.z_scale = 5.0;
//#line 49 "../cfg/Follower.cfg"
      __param_descriptions__.push_back(FollowerConfig::AbstractParamDescriptionConstPtr(new FollowerConfig::ParamDescription<double>("z_scale", "double", 0, "The scaling factor for rotational robot speed.", "", &FollowerConfig::z_scale)));
//#line 239 "/opt/ros/electric/stacks/driver_common/dynamic_reconfigure/templates/ConfigType.h"
    
      for (std::vector<FollowerConfig::AbstractParamDescriptionConstPtr>::const_iterator i = __param_descriptions__.begin(); i != __param_descriptions__.end(); i++)
        __description_message__.parameters.push_back(**i);
      __max__.__toMessage__(__description_message__.max, __param_descriptions__); 
      __min__.__toMessage__(__description_message__.min, __param_descriptions__); 
      __default__.__toMessage__(__description_message__.dflt, __param_descriptions__); 
    }
    std::vector<FollowerConfig::AbstractParamDescriptionConstPtr> __param_descriptions__;
    FollowerConfig __max__;
    FollowerConfig __min__;
    FollowerConfig __default__;
    dynamic_reconfigure::ConfigDescription __description_message__;
    static const FollowerConfigStatics *get_instance()
    {
      // Split this off in a separate function because I know that
      // instance will get initialized the first time get_instance is
      // called, and I am guaranteeing that get_instance gets called at
      // most once.
      static FollowerConfigStatics instance;
      return &instance;
    }
  };

  inline const dynamic_reconfigure::ConfigDescription &FollowerConfig::__getDescriptionMessage__() 
  {
    return __get_statics__()->__description_message__;
  }

  inline const FollowerConfig &FollowerConfig::__getDefault__()
  {
    return __get_statics__()->__default__;
  }
  
  inline const FollowerConfig &FollowerConfig::__getMax__()
  {
    return __get_statics__()->__max__;
  }
  
  inline const FollowerConfig &FollowerConfig::__getMin__()
  {
    return __get_statics__()->__min__;
  }
  
  inline const std::vector<FollowerConfig::AbstractParamDescriptionConstPtr> &FollowerConfig::__getParamDescriptions__()
  {
    return __get_statics__()->__param_descriptions__;
  }

  inline const FollowerConfigStatics *FollowerConfig::__get_statics__()
  {
    const static FollowerConfigStatics *statics;
  
    if (statics) // Common case
      return statics;

    boost::mutex::scoped_lock lock(dynamic_reconfigure::__init_mutex__);

    if (statics) // In case we lost a race.
      return statics;

    statics = FollowerConfigStatics::get_instance();
    
    return statics;
  }


}

#endif // __FOLLOWERRECONFIGURATOR_H__
