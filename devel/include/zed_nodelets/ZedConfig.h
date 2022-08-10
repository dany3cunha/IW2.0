//#line 2 "/opt/ros/melodic/share/dynamic_reconfigure/cmake/../templates/ConfigType.h.template"
// *********************************************************
//
// File autogenerated for the zed_nodelets package
// by the dynamic_reconfigure package.
// Please do not edit.
//
// ********************************************************/

#ifndef __zed_nodelets__ZEDCONFIG_H__
#define __zed_nodelets__ZEDCONFIG_H__

#if __cplusplus >= 201103L
#define DYNAMIC_RECONFIGURE_FINAL final
#else
#define DYNAMIC_RECONFIGURE_FINAL
#endif

#include <dynamic_reconfigure/config_tools.h>
#include <limits>
#include <ros/node_handle.h>
#include <dynamic_reconfigure/ConfigDescription.h>
#include <dynamic_reconfigure/ParamDescription.h>
#include <dynamic_reconfigure/Group.h>
#include <dynamic_reconfigure/config_init_mutex.h>
#include <boost/any.hpp>

namespace zed_nodelets
{
  class ZedConfigStatics;

  class ZedConfig
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

      virtual void clamp(ZedConfig &config, const ZedConfig &max, const ZedConfig &min) const = 0;
      virtual void calcLevel(uint32_t &level, const ZedConfig &config1, const ZedConfig &config2) const = 0;
      virtual void fromServer(const ros::NodeHandle &nh, ZedConfig &config) const = 0;
      virtual void toServer(const ros::NodeHandle &nh, const ZedConfig &config) const = 0;
      virtual bool fromMessage(const dynamic_reconfigure::Config &msg, ZedConfig &config) const = 0;
      virtual void toMessage(dynamic_reconfigure::Config &msg, const ZedConfig &config) const = 0;
      virtual void getValue(const ZedConfig &config, boost::any &val) const = 0;
    };

    typedef boost::shared_ptr<AbstractParamDescription> AbstractParamDescriptionPtr;
    typedef boost::shared_ptr<const AbstractParamDescription> AbstractParamDescriptionConstPtr;

    // Final keyword added to class because it has virtual methods and inherits
    // from a class with a non-virtual destructor.
    template <class T>
    class ParamDescription DYNAMIC_RECONFIGURE_FINAL : public AbstractParamDescription
    {
    public:
      ParamDescription(std::string a_name, std::string a_type, uint32_t a_level,
          std::string a_description, std::string a_edit_method, T ZedConfig::* a_f) :
        AbstractParamDescription(a_name, a_type, a_level, a_description, a_edit_method),
        field(a_f)
      {}

      T ZedConfig::* field;

      virtual void clamp(ZedConfig &config, const ZedConfig &max, const ZedConfig &min) const
      {
        if (config.*field > max.*field)
          config.*field = max.*field;

        if (config.*field < min.*field)
          config.*field = min.*field;
      }

      virtual void calcLevel(uint32_t &comb_level, const ZedConfig &config1, const ZedConfig &config2) const
      {
        if (config1.*field != config2.*field)
          comb_level |= level;
      }

      virtual void fromServer(const ros::NodeHandle &nh, ZedConfig &config) const
      {
        nh.getParam(name, config.*field);
      }

      virtual void toServer(const ros::NodeHandle &nh, const ZedConfig &config) const
      {
        nh.setParam(name, config.*field);
      }

      virtual bool fromMessage(const dynamic_reconfigure::Config &msg, ZedConfig &config) const
      {
        return dynamic_reconfigure::ConfigTools::getParameter(msg, name, config.*field);
      }

      virtual void toMessage(dynamic_reconfigure::Config &msg, const ZedConfig &config) const
      {
        dynamic_reconfigure::ConfigTools::appendParameter(msg, name, config.*field);
      }

      virtual void getValue(const ZedConfig &config, boost::any &val) const
      {
        val = config.*field;
      }
    };

    class AbstractGroupDescription : public dynamic_reconfigure::Group
    {
      public:
      AbstractGroupDescription(std::string n, std::string t, int p, int i, bool s)
      {
        name = n;
        type = t;
        parent = p;
        state = s;
        id = i;
      }

      std::vector<AbstractParamDescriptionConstPtr> abstract_parameters;
      bool state;

      virtual void toMessage(dynamic_reconfigure::Config &msg, const boost::any &config) const = 0;
      virtual bool fromMessage(const dynamic_reconfigure::Config &msg, boost::any &config) const =0;
      virtual void updateParams(boost::any &cfg, ZedConfig &top) const= 0;
      virtual void setInitialState(boost::any &cfg) const = 0;


      void convertParams()
      {
        for(std::vector<AbstractParamDescriptionConstPtr>::const_iterator i = abstract_parameters.begin(); i != abstract_parameters.end(); ++i)
        {
          parameters.push_back(dynamic_reconfigure::ParamDescription(**i));
        }
      }
    };

    typedef boost::shared_ptr<AbstractGroupDescription> AbstractGroupDescriptionPtr;
    typedef boost::shared_ptr<const AbstractGroupDescription> AbstractGroupDescriptionConstPtr;

    // Final keyword added to class because it has virtual methods and inherits
    // from a class with a non-virtual destructor.
    template<class T, class PT>
    class GroupDescription DYNAMIC_RECONFIGURE_FINAL : public AbstractGroupDescription
    {
    public:
      GroupDescription(std::string a_name, std::string a_type, int a_parent, int a_id, bool a_s, T PT::* a_f) : AbstractGroupDescription(a_name, a_type, a_parent, a_id, a_s), field(a_f)
      {
      }

      GroupDescription(const GroupDescription<T, PT>& g): AbstractGroupDescription(g.name, g.type, g.parent, g.id, g.state), field(g.field), groups(g.groups)
      {
        parameters = g.parameters;
        abstract_parameters = g.abstract_parameters;
      }

      virtual bool fromMessage(const dynamic_reconfigure::Config &msg, boost::any &cfg) const
      {
        PT* config = boost::any_cast<PT*>(cfg);
        if(!dynamic_reconfigure::ConfigTools::getGroupState(msg, name, (*config).*field))
          return false;

        for(std::vector<AbstractGroupDescriptionConstPtr>::const_iterator i = groups.begin(); i != groups.end(); ++i)
        {
          boost::any n = &((*config).*field);
          if(!(*i)->fromMessage(msg, n))
            return false;
        }

        return true;
      }

      virtual void setInitialState(boost::any &cfg) const
      {
        PT* config = boost::any_cast<PT*>(cfg);
        T* group = &((*config).*field);
        group->state = state;

        for(std::vector<AbstractGroupDescriptionConstPtr>::const_iterator i = groups.begin(); i != groups.end(); ++i)
        {
          boost::any n = boost::any(&((*config).*field));
          (*i)->setInitialState(n);
        }

      }

      virtual void updateParams(boost::any &cfg, ZedConfig &top) const
      {
        PT* config = boost::any_cast<PT*>(cfg);

        T* f = &((*config).*field);
        f->setParams(top, abstract_parameters);

        for(std::vector<AbstractGroupDescriptionConstPtr>::const_iterator i = groups.begin(); i != groups.end(); ++i)
        {
          boost::any n = &((*config).*field);
          (*i)->updateParams(n, top);
        }
      }

      virtual void toMessage(dynamic_reconfigure::Config &msg, const boost::any &cfg) const
      {
        const PT config = boost::any_cast<PT>(cfg);
        dynamic_reconfigure::ConfigTools::appendGroup<T>(msg, name, id, parent, config.*field);

        for(std::vector<AbstractGroupDescriptionConstPtr>::const_iterator i = groups.begin(); i != groups.end(); ++i)
        {
          (*i)->toMessage(msg, config.*field);
        }
      }

      T PT::* field;
      std::vector<ZedConfig::AbstractGroupDescriptionConstPtr> groups;
    };

class DEFAULT
{
  public:
    DEFAULT()
    {
      state = true;
      name = "Default";
    }

    void setParams(ZedConfig &config, const std::vector<AbstractParamDescriptionConstPtr> params)
    {
      for (std::vector<AbstractParamDescriptionConstPtr>::const_iterator _i = params.begin(); _i != params.end(); ++_i)
      {
        boost::any val;
        (*_i)->getValue(config, val);


      }
    }

    

    bool state;
    std::string name;

    class GENERAL
{
  public:
    GENERAL()
    {
      state = true;
      name = "general";
    }

    void setParams(ZedConfig &config, const std::vector<AbstractParamDescriptionConstPtr> params)
    {
      for (std::vector<AbstractParamDescriptionConstPtr>::const_iterator _i = params.begin(); _i != params.end(); ++_i)
      {
        boost::any val;
        (*_i)->getValue(config, val);

        if("pub_frame_rate"==(*_i)->name){pub_frame_rate = boost::any_cast<double>(val);}
      }
    }

    double pub_frame_rate;

    bool state;
    std::string name;

    
}general;

class DEPTH
{
  public:
    DEPTH()
    {
      state = true;
      name = "depth";
    }

    void setParams(ZedConfig &config, const std::vector<AbstractParamDescriptionConstPtr> params)
    {
      for (std::vector<AbstractParamDescriptionConstPtr>::const_iterator _i = params.begin(); _i != params.end(); ++_i)
      {
        boost::any val;
        (*_i)->getValue(config, val);

        if("depth_confidence"==(*_i)->name){depth_confidence = boost::any_cast<int>(val);}
        if("depth_texture_conf"==(*_i)->name){depth_texture_conf = boost::any_cast<int>(val);}
        if("point_cloud_freq"==(*_i)->name){point_cloud_freq = boost::any_cast<double>(val);}
      }
    }

    int depth_confidence;
int depth_texture_conf;
double point_cloud_freq;

    bool state;
    std::string name;

    
}depth;

class VIDEO
{
  public:
    VIDEO()
    {
      state = true;
      name = "video";
    }

    void setParams(ZedConfig &config, const std::vector<AbstractParamDescriptionConstPtr> params)
    {
      for (std::vector<AbstractParamDescriptionConstPtr>::const_iterator _i = params.begin(); _i != params.end(); ++_i)
      {
        boost::any val;
        (*_i)->getValue(config, val);

        if("brightness"==(*_i)->name){brightness = boost::any_cast<int>(val);}
        if("contrast"==(*_i)->name){contrast = boost::any_cast<int>(val);}
        if("hue"==(*_i)->name){hue = boost::any_cast<int>(val);}
        if("saturation"==(*_i)->name){saturation = boost::any_cast<int>(val);}
        if("sharpness"==(*_i)->name){sharpness = boost::any_cast<int>(val);}
        if("gamma"==(*_i)->name){gamma = boost::any_cast<int>(val);}
        if("auto_exposure_gain"==(*_i)->name){auto_exposure_gain = boost::any_cast<bool>(val);}
        if("gain"==(*_i)->name){gain = boost::any_cast<int>(val);}
        if("exposure"==(*_i)->name){exposure = boost::any_cast<int>(val);}
        if("auto_whitebalance"==(*_i)->name){auto_whitebalance = boost::any_cast<bool>(val);}
        if("whitebalance_temperature"==(*_i)->name){whitebalance_temperature = boost::any_cast<int>(val);}
      }
    }

    int brightness;
int contrast;
int hue;
int saturation;
int sharpness;
int gamma;
bool auto_exposure_gain;
int gain;
int exposure;
bool auto_whitebalance;
int whitebalance_temperature;

    bool state;
    std::string name;

    
}video;

}groups;



//#line 9 "/home/pc/Documents/GitHub/IW2.0/src/zed-ros-wrapper/zed_nodelets/cfg/Zed.cfg"
      double pub_frame_rate;
//#line 12 "/home/pc/Documents/GitHub/IW2.0/src/zed-ros-wrapper/zed_nodelets/cfg/Zed.cfg"
      int depth_confidence;
//#line 13 "/home/pc/Documents/GitHub/IW2.0/src/zed-ros-wrapper/zed_nodelets/cfg/Zed.cfg"
      int depth_texture_conf;
//#line 14 "/home/pc/Documents/GitHub/IW2.0/src/zed-ros-wrapper/zed_nodelets/cfg/Zed.cfg"
      double point_cloud_freq;
//#line 17 "/home/pc/Documents/GitHub/IW2.0/src/zed-ros-wrapper/zed_nodelets/cfg/Zed.cfg"
      int brightness;
//#line 18 "/home/pc/Documents/GitHub/IW2.0/src/zed-ros-wrapper/zed_nodelets/cfg/Zed.cfg"
      int contrast;
//#line 19 "/home/pc/Documents/GitHub/IW2.0/src/zed-ros-wrapper/zed_nodelets/cfg/Zed.cfg"
      int hue;
//#line 20 "/home/pc/Documents/GitHub/IW2.0/src/zed-ros-wrapper/zed_nodelets/cfg/Zed.cfg"
      int saturation;
//#line 21 "/home/pc/Documents/GitHub/IW2.0/src/zed-ros-wrapper/zed_nodelets/cfg/Zed.cfg"
      int sharpness;
//#line 22 "/home/pc/Documents/GitHub/IW2.0/src/zed-ros-wrapper/zed_nodelets/cfg/Zed.cfg"
      int gamma;
//#line 23 "/home/pc/Documents/GitHub/IW2.0/src/zed-ros-wrapper/zed_nodelets/cfg/Zed.cfg"
      bool auto_exposure_gain;
//#line 24 "/home/pc/Documents/GitHub/IW2.0/src/zed-ros-wrapper/zed_nodelets/cfg/Zed.cfg"
      int gain;
//#line 25 "/home/pc/Documents/GitHub/IW2.0/src/zed-ros-wrapper/zed_nodelets/cfg/Zed.cfg"
      int exposure;
//#line 26 "/home/pc/Documents/GitHub/IW2.0/src/zed-ros-wrapper/zed_nodelets/cfg/Zed.cfg"
      bool auto_whitebalance;
//#line 27 "/home/pc/Documents/GitHub/IW2.0/src/zed-ros-wrapper/zed_nodelets/cfg/Zed.cfg"
      int whitebalance_temperature;
//#line 228 "/opt/ros/melodic/share/dynamic_reconfigure/cmake/../templates/ConfigType.h.template"

    bool __fromMessage__(dynamic_reconfigure::Config &msg)
    {
      const std::vector<AbstractParamDescriptionConstPtr> &__param_descriptions__ = __getParamDescriptions__();
      const std::vector<AbstractGroupDescriptionConstPtr> &__group_descriptions__ = __getGroupDescriptions__();

      int count = 0;
      for (std::vector<AbstractParamDescriptionConstPtr>::const_iterator i = __param_descriptions__.begin(); i != __param_descriptions__.end(); ++i)
        if ((*i)->fromMessage(msg, *this))
          count++;

      for (std::vector<AbstractGroupDescriptionConstPtr>::const_iterator i = __group_descriptions__.begin(); i != __group_descriptions__.end(); i ++)
      {
        if ((*i)->id == 0)
        {
          boost::any n = boost::any(this);
          (*i)->updateParams(n, *this);
          (*i)->fromMessage(msg, n);
        }
      }

      if (count != dynamic_reconfigure::ConfigTools::size(msg))
      {
        ROS_ERROR("ZedConfig::__fromMessage__ called with an unexpected parameter.");
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
    void __toMessage__(dynamic_reconfigure::Config &msg, const std::vector<AbstractParamDescriptionConstPtr> &__param_descriptions__, const std::vector<AbstractGroupDescriptionConstPtr> &__group_descriptions__) const
    {
      dynamic_reconfigure::ConfigTools::clear(msg);
      for (std::vector<AbstractParamDescriptionConstPtr>::const_iterator i = __param_descriptions__.begin(); i != __param_descriptions__.end(); ++i)
        (*i)->toMessage(msg, *this);

      for (std::vector<AbstractGroupDescriptionConstPtr>::const_iterator i = __group_descriptions__.begin(); i != __group_descriptions__.end(); ++i)
      {
        if((*i)->id == 0)
        {
          (*i)->toMessage(msg, *this);
        }
      }
    }

    void __toMessage__(dynamic_reconfigure::Config &msg) const
    {
      const std::vector<AbstractParamDescriptionConstPtr> &__param_descriptions__ = __getParamDescriptions__();
      const std::vector<AbstractGroupDescriptionConstPtr> &__group_descriptions__ = __getGroupDescriptions__();
      __toMessage__(msg, __param_descriptions__, __group_descriptions__);
    }

    void __toServer__(const ros::NodeHandle &nh) const
    {
      const std::vector<AbstractParamDescriptionConstPtr> &__param_descriptions__ = __getParamDescriptions__();
      for (std::vector<AbstractParamDescriptionConstPtr>::const_iterator i = __param_descriptions__.begin(); i != __param_descriptions__.end(); ++i)
        (*i)->toServer(nh, *this);
    }

    void __fromServer__(const ros::NodeHandle &nh)
    {
      static bool setup=false;

      const std::vector<AbstractParamDescriptionConstPtr> &__param_descriptions__ = __getParamDescriptions__();
      for (std::vector<AbstractParamDescriptionConstPtr>::const_iterator i = __param_descriptions__.begin(); i != __param_descriptions__.end(); ++i)
        (*i)->fromServer(nh, *this);

      const std::vector<AbstractGroupDescriptionConstPtr> &__group_descriptions__ = __getGroupDescriptions__();
      for (std::vector<AbstractGroupDescriptionConstPtr>::const_iterator i = __group_descriptions__.begin(); i != __group_descriptions__.end(); i++){
        if (!setup && (*i)->id == 0) {
          setup = true;
          boost::any n = boost::any(this);
          (*i)->setInitialState(n);
        }
      }
    }

    void __clamp__()
    {
      const std::vector<AbstractParamDescriptionConstPtr> &__param_descriptions__ = __getParamDescriptions__();
      const ZedConfig &__max__ = __getMax__();
      const ZedConfig &__min__ = __getMin__();
      for (std::vector<AbstractParamDescriptionConstPtr>::const_iterator i = __param_descriptions__.begin(); i != __param_descriptions__.end(); ++i)
        (*i)->clamp(*this, __max__, __min__);
    }

    uint32_t __level__(const ZedConfig &config) const
    {
      const std::vector<AbstractParamDescriptionConstPtr> &__param_descriptions__ = __getParamDescriptions__();
      uint32_t level = 0;
      for (std::vector<AbstractParamDescriptionConstPtr>::const_iterator i = __param_descriptions__.begin(); i != __param_descriptions__.end(); ++i)
        (*i)->calcLevel(level, config, *this);
      return level;
    }

    static const dynamic_reconfigure::ConfigDescription &__getDescriptionMessage__();
    static const ZedConfig &__getDefault__();
    static const ZedConfig &__getMax__();
    static const ZedConfig &__getMin__();
    static const std::vector<AbstractParamDescriptionConstPtr> &__getParamDescriptions__();
    static const std::vector<AbstractGroupDescriptionConstPtr> &__getGroupDescriptions__();

  private:
    static const ZedConfigStatics *__get_statics__();
  };

  template <> // Max and min are ignored for strings.
  inline void ZedConfig::ParamDescription<std::string>::clamp(ZedConfig &config, const ZedConfig &max, const ZedConfig &min) const
  {
    (void) config;
    (void) min;
    (void) max;
    return;
  }

  class ZedConfigStatics
  {
    friend class ZedConfig;

    ZedConfigStatics()
    {
ZedConfig::GroupDescription<ZedConfig::DEFAULT, ZedConfig> Default("Default", "", 0, 0, true, &ZedConfig::groups);
ZedConfig::GroupDescription<ZedConfig::DEFAULT::GENERAL, ZedConfig::DEFAULT> general("general", "", 0, 1, true, &ZedConfig::DEFAULT::general);
//#line 9 "/home/pc/Documents/GitHub/IW2.0/src/zed-ros-wrapper/zed_nodelets/cfg/Zed.cfg"
      __min__.pub_frame_rate = 0.1;
//#line 9 "/home/pc/Documents/GitHub/IW2.0/src/zed-ros-wrapper/zed_nodelets/cfg/Zed.cfg"
      __max__.pub_frame_rate = 60.0;
//#line 9 "/home/pc/Documents/GitHub/IW2.0/src/zed-ros-wrapper/zed_nodelets/cfg/Zed.cfg"
      __default__.pub_frame_rate = 15.0;
//#line 9 "/home/pc/Documents/GitHub/IW2.0/src/zed-ros-wrapper/zed_nodelets/cfg/Zed.cfg"
      general.abstract_parameters.push_back(ZedConfig::AbstractParamDescriptionConstPtr(new ZedConfig::ParamDescription<double>("pub_frame_rate", "double", 0, "Video and Depth data frequency", "", &ZedConfig::pub_frame_rate)));
//#line 9 "/home/pc/Documents/GitHub/IW2.0/src/zed-ros-wrapper/zed_nodelets/cfg/Zed.cfg"
      __param_descriptions__.push_back(ZedConfig::AbstractParamDescriptionConstPtr(new ZedConfig::ParamDescription<double>("pub_frame_rate", "double", 0, "Video and Depth data frequency", "", &ZedConfig::pub_frame_rate)));
//#line 124 "/opt/ros/melodic/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py"
      general.convertParams();
//#line 124 "/opt/ros/melodic/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py"
      Default.groups.push_back(ZedConfig::AbstractGroupDescriptionConstPtr(new ZedConfig::GroupDescription<ZedConfig::DEFAULT::GENERAL, ZedConfig::DEFAULT>(general)));
//#line 124 "/opt/ros/melodic/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py"
      __group_descriptions__.push_back(ZedConfig::AbstractGroupDescriptionConstPtr(new ZedConfig::GroupDescription<ZedConfig::DEFAULT::GENERAL, ZedConfig::DEFAULT>(general)));
ZedConfig::GroupDescription<ZedConfig::DEFAULT::DEPTH, ZedConfig::DEFAULT> depth("depth", "", 0, 2, true, &ZedConfig::DEFAULT::depth);
//#line 12 "/home/pc/Documents/GitHub/IW2.0/src/zed-ros-wrapper/zed_nodelets/cfg/Zed.cfg"
      __min__.depth_confidence = 1;
//#line 12 "/home/pc/Documents/GitHub/IW2.0/src/zed-ros-wrapper/zed_nodelets/cfg/Zed.cfg"
      __max__.depth_confidence = 100;
//#line 12 "/home/pc/Documents/GitHub/IW2.0/src/zed-ros-wrapper/zed_nodelets/cfg/Zed.cfg"
      __default__.depth_confidence = 50;
//#line 12 "/home/pc/Documents/GitHub/IW2.0/src/zed-ros-wrapper/zed_nodelets/cfg/Zed.cfg"
      depth.abstract_parameters.push_back(ZedConfig::AbstractParamDescriptionConstPtr(new ZedConfig::ParamDescription<int>("depth_confidence", "int", 1, "Depth confidence threshold", "", &ZedConfig::depth_confidence)));
//#line 12 "/home/pc/Documents/GitHub/IW2.0/src/zed-ros-wrapper/zed_nodelets/cfg/Zed.cfg"
      __param_descriptions__.push_back(ZedConfig::AbstractParamDescriptionConstPtr(new ZedConfig::ParamDescription<int>("depth_confidence", "int", 1, "Depth confidence threshold", "", &ZedConfig::depth_confidence)));
//#line 13 "/home/pc/Documents/GitHub/IW2.0/src/zed-ros-wrapper/zed_nodelets/cfg/Zed.cfg"
      __min__.depth_texture_conf = 1;
//#line 13 "/home/pc/Documents/GitHub/IW2.0/src/zed-ros-wrapper/zed_nodelets/cfg/Zed.cfg"
      __max__.depth_texture_conf = 100;
//#line 13 "/home/pc/Documents/GitHub/IW2.0/src/zed-ros-wrapper/zed_nodelets/cfg/Zed.cfg"
      __default__.depth_texture_conf = 100;
//#line 13 "/home/pc/Documents/GitHub/IW2.0/src/zed-ros-wrapper/zed_nodelets/cfg/Zed.cfg"
      depth.abstract_parameters.push_back(ZedConfig::AbstractParamDescriptionConstPtr(new ZedConfig::ParamDescription<int>("depth_texture_conf", "int", 2, "Texture confidence threshold", "", &ZedConfig::depth_texture_conf)));
//#line 13 "/home/pc/Documents/GitHub/IW2.0/src/zed-ros-wrapper/zed_nodelets/cfg/Zed.cfg"
      __param_descriptions__.push_back(ZedConfig::AbstractParamDescriptionConstPtr(new ZedConfig::ParamDescription<int>("depth_texture_conf", "int", 2, "Texture confidence threshold", "", &ZedConfig::depth_texture_conf)));
//#line 14 "/home/pc/Documents/GitHub/IW2.0/src/zed-ros-wrapper/zed_nodelets/cfg/Zed.cfg"
      __min__.point_cloud_freq = 0.1;
//#line 14 "/home/pc/Documents/GitHub/IW2.0/src/zed-ros-wrapper/zed_nodelets/cfg/Zed.cfg"
      __max__.point_cloud_freq = 60.0;
//#line 14 "/home/pc/Documents/GitHub/IW2.0/src/zed-ros-wrapper/zed_nodelets/cfg/Zed.cfg"
      __default__.point_cloud_freq = 15.0;
//#line 14 "/home/pc/Documents/GitHub/IW2.0/src/zed-ros-wrapper/zed_nodelets/cfg/Zed.cfg"
      depth.abstract_parameters.push_back(ZedConfig::AbstractParamDescriptionConstPtr(new ZedConfig::ParamDescription<double>("point_cloud_freq", "double", 3, "Point cloud frequency", "", &ZedConfig::point_cloud_freq)));
//#line 14 "/home/pc/Documents/GitHub/IW2.0/src/zed-ros-wrapper/zed_nodelets/cfg/Zed.cfg"
      __param_descriptions__.push_back(ZedConfig::AbstractParamDescriptionConstPtr(new ZedConfig::ParamDescription<double>("point_cloud_freq", "double", 3, "Point cloud frequency", "", &ZedConfig::point_cloud_freq)));
//#line 124 "/opt/ros/melodic/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py"
      depth.convertParams();
//#line 124 "/opt/ros/melodic/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py"
      Default.groups.push_back(ZedConfig::AbstractGroupDescriptionConstPtr(new ZedConfig::GroupDescription<ZedConfig::DEFAULT::DEPTH, ZedConfig::DEFAULT>(depth)));
//#line 124 "/opt/ros/melodic/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py"
      __group_descriptions__.push_back(ZedConfig::AbstractGroupDescriptionConstPtr(new ZedConfig::GroupDescription<ZedConfig::DEFAULT::DEPTH, ZedConfig::DEFAULT>(depth)));
ZedConfig::GroupDescription<ZedConfig::DEFAULT::VIDEO, ZedConfig::DEFAULT> video("video", "", 0, 3, true, &ZedConfig::DEFAULT::video);
//#line 17 "/home/pc/Documents/GitHub/IW2.0/src/zed-ros-wrapper/zed_nodelets/cfg/Zed.cfg"
      __min__.brightness = 0;
//#line 17 "/home/pc/Documents/GitHub/IW2.0/src/zed-ros-wrapper/zed_nodelets/cfg/Zed.cfg"
      __max__.brightness = 8;
//#line 17 "/home/pc/Documents/GitHub/IW2.0/src/zed-ros-wrapper/zed_nodelets/cfg/Zed.cfg"
      __default__.brightness = 4;
//#line 17 "/home/pc/Documents/GitHub/IW2.0/src/zed-ros-wrapper/zed_nodelets/cfg/Zed.cfg"
      video.abstract_parameters.push_back(ZedConfig::AbstractParamDescriptionConstPtr(new ZedConfig::ParamDescription<int>("brightness", "int", 4, "Defines the brightness control", "", &ZedConfig::brightness)));
//#line 17 "/home/pc/Documents/GitHub/IW2.0/src/zed-ros-wrapper/zed_nodelets/cfg/Zed.cfg"
      __param_descriptions__.push_back(ZedConfig::AbstractParamDescriptionConstPtr(new ZedConfig::ParamDescription<int>("brightness", "int", 4, "Defines the brightness control", "", &ZedConfig::brightness)));
//#line 18 "/home/pc/Documents/GitHub/IW2.0/src/zed-ros-wrapper/zed_nodelets/cfg/Zed.cfg"
      __min__.contrast = 0;
//#line 18 "/home/pc/Documents/GitHub/IW2.0/src/zed-ros-wrapper/zed_nodelets/cfg/Zed.cfg"
      __max__.contrast = 8;
//#line 18 "/home/pc/Documents/GitHub/IW2.0/src/zed-ros-wrapper/zed_nodelets/cfg/Zed.cfg"
      __default__.contrast = 4;
//#line 18 "/home/pc/Documents/GitHub/IW2.0/src/zed-ros-wrapper/zed_nodelets/cfg/Zed.cfg"
      video.abstract_parameters.push_back(ZedConfig::AbstractParamDescriptionConstPtr(new ZedConfig::ParamDescription<int>("contrast", "int", 5, "Defines the contrast control", "", &ZedConfig::contrast)));
//#line 18 "/home/pc/Documents/GitHub/IW2.0/src/zed-ros-wrapper/zed_nodelets/cfg/Zed.cfg"
      __param_descriptions__.push_back(ZedConfig::AbstractParamDescriptionConstPtr(new ZedConfig::ParamDescription<int>("contrast", "int", 5, "Defines the contrast control", "", &ZedConfig::contrast)));
//#line 19 "/home/pc/Documents/GitHub/IW2.0/src/zed-ros-wrapper/zed_nodelets/cfg/Zed.cfg"
      __min__.hue = 0;
//#line 19 "/home/pc/Documents/GitHub/IW2.0/src/zed-ros-wrapper/zed_nodelets/cfg/Zed.cfg"
      __max__.hue = 11;
//#line 19 "/home/pc/Documents/GitHub/IW2.0/src/zed-ros-wrapper/zed_nodelets/cfg/Zed.cfg"
      __default__.hue = 0;
//#line 19 "/home/pc/Documents/GitHub/IW2.0/src/zed-ros-wrapper/zed_nodelets/cfg/Zed.cfg"
      video.abstract_parameters.push_back(ZedConfig::AbstractParamDescriptionConstPtr(new ZedConfig::ParamDescription<int>("hue", "int", 6, "Defines the hue control", "", &ZedConfig::hue)));
//#line 19 "/home/pc/Documents/GitHub/IW2.0/src/zed-ros-wrapper/zed_nodelets/cfg/Zed.cfg"
      __param_descriptions__.push_back(ZedConfig::AbstractParamDescriptionConstPtr(new ZedConfig::ParamDescription<int>("hue", "int", 6, "Defines the hue control", "", &ZedConfig::hue)));
//#line 20 "/home/pc/Documents/GitHub/IW2.0/src/zed-ros-wrapper/zed_nodelets/cfg/Zed.cfg"
      __min__.saturation = 0;
//#line 20 "/home/pc/Documents/GitHub/IW2.0/src/zed-ros-wrapper/zed_nodelets/cfg/Zed.cfg"
      __max__.saturation = 8;
//#line 20 "/home/pc/Documents/GitHub/IW2.0/src/zed-ros-wrapper/zed_nodelets/cfg/Zed.cfg"
      __default__.saturation = 4;
//#line 20 "/home/pc/Documents/GitHub/IW2.0/src/zed-ros-wrapper/zed_nodelets/cfg/Zed.cfg"
      video.abstract_parameters.push_back(ZedConfig::AbstractParamDescriptionConstPtr(new ZedConfig::ParamDescription<int>("saturation", "int", 7, "Defines the saturation control", "", &ZedConfig::saturation)));
//#line 20 "/home/pc/Documents/GitHub/IW2.0/src/zed-ros-wrapper/zed_nodelets/cfg/Zed.cfg"
      __param_descriptions__.push_back(ZedConfig::AbstractParamDescriptionConstPtr(new ZedConfig::ParamDescription<int>("saturation", "int", 7, "Defines the saturation control", "", &ZedConfig::saturation)));
//#line 21 "/home/pc/Documents/GitHub/IW2.0/src/zed-ros-wrapper/zed_nodelets/cfg/Zed.cfg"
      __min__.sharpness = 0;
//#line 21 "/home/pc/Documents/GitHub/IW2.0/src/zed-ros-wrapper/zed_nodelets/cfg/Zed.cfg"
      __max__.sharpness = 8;
//#line 21 "/home/pc/Documents/GitHub/IW2.0/src/zed-ros-wrapper/zed_nodelets/cfg/Zed.cfg"
      __default__.sharpness = 4;
//#line 21 "/home/pc/Documents/GitHub/IW2.0/src/zed-ros-wrapper/zed_nodelets/cfg/Zed.cfg"
      video.abstract_parameters.push_back(ZedConfig::AbstractParamDescriptionConstPtr(new ZedConfig::ParamDescription<int>("sharpness", "int", 8, "Defines the digital sharpness control", "", &ZedConfig::sharpness)));
//#line 21 "/home/pc/Documents/GitHub/IW2.0/src/zed-ros-wrapper/zed_nodelets/cfg/Zed.cfg"
      __param_descriptions__.push_back(ZedConfig::AbstractParamDescriptionConstPtr(new ZedConfig::ParamDescription<int>("sharpness", "int", 8, "Defines the digital sharpness control", "", &ZedConfig::sharpness)));
//#line 22 "/home/pc/Documents/GitHub/IW2.0/src/zed-ros-wrapper/zed_nodelets/cfg/Zed.cfg"
      __min__.gamma = 1;
//#line 22 "/home/pc/Documents/GitHub/IW2.0/src/zed-ros-wrapper/zed_nodelets/cfg/Zed.cfg"
      __max__.gamma = 9;
//#line 22 "/home/pc/Documents/GitHub/IW2.0/src/zed-ros-wrapper/zed_nodelets/cfg/Zed.cfg"
      __default__.gamma = 8;
//#line 22 "/home/pc/Documents/GitHub/IW2.0/src/zed-ros-wrapper/zed_nodelets/cfg/Zed.cfg"
      video.abstract_parameters.push_back(ZedConfig::AbstractParamDescriptionConstPtr(new ZedConfig::ParamDescription<int>("gamma", "int", 9, "Defines the gamma control", "", &ZedConfig::gamma)));
//#line 22 "/home/pc/Documents/GitHub/IW2.0/src/zed-ros-wrapper/zed_nodelets/cfg/Zed.cfg"
      __param_descriptions__.push_back(ZedConfig::AbstractParamDescriptionConstPtr(new ZedConfig::ParamDescription<int>("gamma", "int", 9, "Defines the gamma control", "", &ZedConfig::gamma)));
//#line 23 "/home/pc/Documents/GitHub/IW2.0/src/zed-ros-wrapper/zed_nodelets/cfg/Zed.cfg"
      __min__.auto_exposure_gain = 0;
//#line 23 "/home/pc/Documents/GitHub/IW2.0/src/zed-ros-wrapper/zed_nodelets/cfg/Zed.cfg"
      __max__.auto_exposure_gain = 1;
//#line 23 "/home/pc/Documents/GitHub/IW2.0/src/zed-ros-wrapper/zed_nodelets/cfg/Zed.cfg"
      __default__.auto_exposure_gain = 1;
//#line 23 "/home/pc/Documents/GitHub/IW2.0/src/zed-ros-wrapper/zed_nodelets/cfg/Zed.cfg"
      video.abstract_parameters.push_back(ZedConfig::AbstractParamDescriptionConstPtr(new ZedConfig::ParamDescription<bool>("auto_exposure_gain", "bool", 10, "Defines if the Gain and Exposure are in automatic mode or not", "", &ZedConfig::auto_exposure_gain)));
//#line 23 "/home/pc/Documents/GitHub/IW2.0/src/zed-ros-wrapper/zed_nodelets/cfg/Zed.cfg"
      __param_descriptions__.push_back(ZedConfig::AbstractParamDescriptionConstPtr(new ZedConfig::ParamDescription<bool>("auto_exposure_gain", "bool", 10, "Defines if the Gain and Exposure are in automatic mode or not", "", &ZedConfig::auto_exposure_gain)));
//#line 24 "/home/pc/Documents/GitHub/IW2.0/src/zed-ros-wrapper/zed_nodelets/cfg/Zed.cfg"
      __min__.gain = 0;
//#line 24 "/home/pc/Documents/GitHub/IW2.0/src/zed-ros-wrapper/zed_nodelets/cfg/Zed.cfg"
      __max__.gain = 100;
//#line 24 "/home/pc/Documents/GitHub/IW2.0/src/zed-ros-wrapper/zed_nodelets/cfg/Zed.cfg"
      __default__.gain = 100;
//#line 24 "/home/pc/Documents/GitHub/IW2.0/src/zed-ros-wrapper/zed_nodelets/cfg/Zed.cfg"
      video.abstract_parameters.push_back(ZedConfig::AbstractParamDescriptionConstPtr(new ZedConfig::ParamDescription<int>("gain", "int", 11, "Defines the gain control", "", &ZedConfig::gain)));
//#line 24 "/home/pc/Documents/GitHub/IW2.0/src/zed-ros-wrapper/zed_nodelets/cfg/Zed.cfg"
      __param_descriptions__.push_back(ZedConfig::AbstractParamDescriptionConstPtr(new ZedConfig::ParamDescription<int>("gain", "int", 11, "Defines the gain control", "", &ZedConfig::gain)));
//#line 25 "/home/pc/Documents/GitHub/IW2.0/src/zed-ros-wrapper/zed_nodelets/cfg/Zed.cfg"
      __min__.exposure = 0;
//#line 25 "/home/pc/Documents/GitHub/IW2.0/src/zed-ros-wrapper/zed_nodelets/cfg/Zed.cfg"
      __max__.exposure = 100;
//#line 25 "/home/pc/Documents/GitHub/IW2.0/src/zed-ros-wrapper/zed_nodelets/cfg/Zed.cfg"
      __default__.exposure = 100;
//#line 25 "/home/pc/Documents/GitHub/IW2.0/src/zed-ros-wrapper/zed_nodelets/cfg/Zed.cfg"
      video.abstract_parameters.push_back(ZedConfig::AbstractParamDescriptionConstPtr(new ZedConfig::ParamDescription<int>("exposure", "int", 12, "Defines the exposure control", "", &ZedConfig::exposure)));
//#line 25 "/home/pc/Documents/GitHub/IW2.0/src/zed-ros-wrapper/zed_nodelets/cfg/Zed.cfg"
      __param_descriptions__.push_back(ZedConfig::AbstractParamDescriptionConstPtr(new ZedConfig::ParamDescription<int>("exposure", "int", 12, "Defines the exposure control", "", &ZedConfig::exposure)));
//#line 26 "/home/pc/Documents/GitHub/IW2.0/src/zed-ros-wrapper/zed_nodelets/cfg/Zed.cfg"
      __min__.auto_whitebalance = 0;
//#line 26 "/home/pc/Documents/GitHub/IW2.0/src/zed-ros-wrapper/zed_nodelets/cfg/Zed.cfg"
      __max__.auto_whitebalance = 1;
//#line 26 "/home/pc/Documents/GitHub/IW2.0/src/zed-ros-wrapper/zed_nodelets/cfg/Zed.cfg"
      __default__.auto_whitebalance = 1;
//#line 26 "/home/pc/Documents/GitHub/IW2.0/src/zed-ros-wrapper/zed_nodelets/cfg/Zed.cfg"
      video.abstract_parameters.push_back(ZedConfig::AbstractParamDescriptionConstPtr(new ZedConfig::ParamDescription<bool>("auto_whitebalance", "bool", 13, "Defines if the White balance is in automatic mode or not", "", &ZedConfig::auto_whitebalance)));
//#line 26 "/home/pc/Documents/GitHub/IW2.0/src/zed-ros-wrapper/zed_nodelets/cfg/Zed.cfg"
      __param_descriptions__.push_back(ZedConfig::AbstractParamDescriptionConstPtr(new ZedConfig::ParamDescription<bool>("auto_whitebalance", "bool", 13, "Defines if the White balance is in automatic mode or not", "", &ZedConfig::auto_whitebalance)));
//#line 27 "/home/pc/Documents/GitHub/IW2.0/src/zed-ros-wrapper/zed_nodelets/cfg/Zed.cfg"
      __min__.whitebalance_temperature = 28;
//#line 27 "/home/pc/Documents/GitHub/IW2.0/src/zed-ros-wrapper/zed_nodelets/cfg/Zed.cfg"
      __max__.whitebalance_temperature = 65;
//#line 27 "/home/pc/Documents/GitHub/IW2.0/src/zed-ros-wrapper/zed_nodelets/cfg/Zed.cfg"
      __default__.whitebalance_temperature = 42;
//#line 27 "/home/pc/Documents/GitHub/IW2.0/src/zed-ros-wrapper/zed_nodelets/cfg/Zed.cfg"
      video.abstract_parameters.push_back(ZedConfig::AbstractParamDescriptionConstPtr(new ZedConfig::ParamDescription<int>("whitebalance_temperature", "int", 14, "Defines the color temperature value (x100)", "", &ZedConfig::whitebalance_temperature)));
//#line 27 "/home/pc/Documents/GitHub/IW2.0/src/zed-ros-wrapper/zed_nodelets/cfg/Zed.cfg"
      __param_descriptions__.push_back(ZedConfig::AbstractParamDescriptionConstPtr(new ZedConfig::ParamDescription<int>("whitebalance_temperature", "int", 14, "Defines the color temperature value (x100)", "", &ZedConfig::whitebalance_temperature)));
//#line 124 "/opt/ros/melodic/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py"
      video.convertParams();
//#line 124 "/opt/ros/melodic/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py"
      Default.groups.push_back(ZedConfig::AbstractGroupDescriptionConstPtr(new ZedConfig::GroupDescription<ZedConfig::DEFAULT::VIDEO, ZedConfig::DEFAULT>(video)));
//#line 124 "/opt/ros/melodic/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py"
      __group_descriptions__.push_back(ZedConfig::AbstractGroupDescriptionConstPtr(new ZedConfig::GroupDescription<ZedConfig::DEFAULT::VIDEO, ZedConfig::DEFAULT>(video)));
//#line 246 "/opt/ros/melodic/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py"
      Default.convertParams();
//#line 246 "/opt/ros/melodic/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py"
      __group_descriptions__.push_back(ZedConfig::AbstractGroupDescriptionConstPtr(new ZedConfig::GroupDescription<ZedConfig::DEFAULT, ZedConfig>(Default)));
//#line 366 "/opt/ros/melodic/share/dynamic_reconfigure/cmake/../templates/ConfigType.h.template"

      for (std::vector<ZedConfig::AbstractGroupDescriptionConstPtr>::const_iterator i = __group_descriptions__.begin(); i != __group_descriptions__.end(); ++i)
      {
        __description_message__.groups.push_back(**i);
      }
      __max__.__toMessage__(__description_message__.max, __param_descriptions__, __group_descriptions__);
      __min__.__toMessage__(__description_message__.min, __param_descriptions__, __group_descriptions__);
      __default__.__toMessage__(__description_message__.dflt, __param_descriptions__, __group_descriptions__);
    }
    std::vector<ZedConfig::AbstractParamDescriptionConstPtr> __param_descriptions__;
    std::vector<ZedConfig::AbstractGroupDescriptionConstPtr> __group_descriptions__;
    ZedConfig __max__;
    ZedConfig __min__;
    ZedConfig __default__;
    dynamic_reconfigure::ConfigDescription __description_message__;

    static const ZedConfigStatics *get_instance()
    {
      // Split this off in a separate function because I know that
      // instance will get initialized the first time get_instance is
      // called, and I am guaranteeing that get_instance gets called at
      // most once.
      static ZedConfigStatics instance;
      return &instance;
    }
  };

  inline const dynamic_reconfigure::ConfigDescription &ZedConfig::__getDescriptionMessage__()
  {
    return __get_statics__()->__description_message__;
  }

  inline const ZedConfig &ZedConfig::__getDefault__()
  {
    return __get_statics__()->__default__;
  }

  inline const ZedConfig &ZedConfig::__getMax__()
  {
    return __get_statics__()->__max__;
  }

  inline const ZedConfig &ZedConfig::__getMin__()
  {
    return __get_statics__()->__min__;
  }

  inline const std::vector<ZedConfig::AbstractParamDescriptionConstPtr> &ZedConfig::__getParamDescriptions__()
  {
    return __get_statics__()->__param_descriptions__;
  }

  inline const std::vector<ZedConfig::AbstractGroupDescriptionConstPtr> &ZedConfig::__getGroupDescriptions__()
  {
    return __get_statics__()->__group_descriptions__;
  }

  inline const ZedConfigStatics *ZedConfig::__get_statics__()
  {
    const static ZedConfigStatics *statics;

    if (statics) // Common case
      return statics;

    boost::mutex::scoped_lock lock(dynamic_reconfigure::__init_mutex__);

    if (statics) // In case we lost a race.
      return statics;

    statics = ZedConfigStatics::get_instance();

    return statics;
  }


}

#undef DYNAMIC_RECONFIGURE_FINAL

#endif // __ZEDRECONFIGURATOR_H__
