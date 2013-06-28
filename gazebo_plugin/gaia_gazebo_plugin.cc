#include <boost/bind.hpp>
#include <gazebo.hh>
#include <physics/physics.hh>
#include <sensors/sensors.hh>
#include <common/common.hh>
#include <stdio.h>

#include "ros/ros.h"
#include "std_msgs/Float64.h"
#include "geometry_msgs/Point.h"
#include "sensor_msgs/LaserScan.h"





namespace gazebo
{   
  class ROSModelPlugin : public ModelPlugin
  {

    public: ROSModelPlugin()
    {

      // Start up ROS
      std::string name = "gaia_gazebo_plugin";
      int argc = 0;
      ros::init(argc, NULL, name);

    }
    public: ~ROSModelPlugin()
    {
      delete this->node;
    }

    public: void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf)
    {
      // Store the pointer to the model
      this->model = _parent;

      // ROS Nodehandle
      this->node = new ros::NodeHandle("~");

      // ROS Subscriber
      this->sub = this->node->subscribe<geometry_msgs::Point>("gaia_driver", 1000, &ROSModelPlugin::ROSCallback, this );

      this->pub = this->node->advertise<sensor_msgs::LaserScan>("gaia_lidar",1000); 



      // Listen to the update event. This event is broadcast every
      // simulation iteration.
      this->updateConnection = event::Events::ConnectWorldUpdateBegin(
          boost::bind(&ROSModelPlugin::OnUpdate, this));

  //Added:
      // Load parameters for this plugin
      if (this->LoadParams(_sdf))
      {
        // testing to see if race condition exists
        gzerr << this->leftWheelJoint->GetAngle(0) << "\n";
        gzerr << this->rightWheelJoint->GetAngle(0) << "\n";
        // Listen to the update event. This event is broadcast every
        // simulation iteration.
        this->updateConnection = event::Events::ConnectWorldUpdateBegin(
            boost::bind(&ROSModelPlugin::OnUpdate, this));
      }
    }

    public: bool LoadParams(sdf::ElementPtr _sdf) 
    {

      // Find controller gain
      if (!_sdf->HasElement("gain"))
      {
        gzerr << "param [gain] not found\n";
        return false;
      }
      else
      {
        // Get sensor name
        this->gain =
          _sdf->GetElement("gain")->GetValueDouble();
      }


      

      // Find sensor name from plugin param
      if (!_sdf->HasElement("ray_sensor"))
      {
        gzerr << "param [ray_sensor] not found\n";
        return false;
      }
      else
      {
        // Get sensor name
        std::string sensorName =
          _sdf->GetElement("ray_sensor")->GetValueString();

        // Get pointer to sensor using the SensorMangaer
        sensors::SensorPtr sensor =
          sensors::SensorManager::Instance()->GetSensor(sensorName);

        if (!sensor)
        {
          gzerr << "sensor by name ["
                << sensorName
                << "] not found in model\n";
          return false;
        }

        this->laser = boost::shared_dynamic_cast<sensors::RaySensor>
          (sensor);
        if (!this->laser)
        {
          gzerr << "laser by name ["
                << sensorName
                << "] not found in model\n";
          return false;
        }
      }

      // Load joints from plugin param
      if (!this->FindJointByParam(_sdf, this->leftWheelJoint,
                             "left_wheel_hinge") ||
          !this->FindJointByParam(_sdf, this->rightWheelJoint,
                             "right_wheel_hinge"))
        return false;

      // success
      return true;
    }

    public: bool FindJointByParam(sdf::ElementPtr _sdf,
                                  physics::JointPtr &_joint,
                                  std::string _param)
    {
      if (!_sdf->HasElement(_param))
      {
        gzerr << "param [" << _param << "] not found\n";
        return false;
      }
      else
      {
        _joint = this->model->GetJoint(
          _sdf->GetElement(_param)->GetValueString());

        if (!_joint)
        {
          gzerr << "joint by name ["
                << _sdf->GetElement(_param)->GetValueString()
                << "] not found in model\n";
          return false;
        }
      }
      return true;
    }


    // Called by the world update start event
    public: void OnUpdate()
    {
      ros::spinOnce();

      sensor_msgs::LaserScan p;

      int i, ja, jb;
      double ra, rb, r, b;
      double intensity;

      this->laser->SetActive(false);

      math::Angle maxAngle = this->laser->GetAngleMax();
      math::Angle minAngle = this->laser->GetAngleMin();

      double maxRange = this->laser->GetRangeMax();
      double minRange = this->laser->GetRangeMin();
      int rayCount = this->laser->GetRayCount();
      int rangeCount = this->laser->GetRangeCount();

      /***************************************************************/
      /*                                                             */
      /*  point scan from laser                                      */
      /*                                                             */
      /***************************************************************/
      // Add Frame Name
      //p.header.frame_id = this->frame_name_;
      //p.header.stamp.sec = _updateTime.sec;
      //p.header.stamp.nsec = _updateTime.nsec;


      double tmp_res_angle = (maxAngle.Radian() - minAngle.Radian())/((double)(rangeCount -1)); // for computing yaw
      p.angle_min = minAngle.Radian();
      p.angle_max = maxAngle.Radian();
      p.angle_increment = tmp_res_angle;
      p.time_increment  = 0; // instantaneous simulator scan
      p.scan_time       = 0; // FIXME: what's this?
      p.range_min = minRange;
      p.range_max = maxRange;
      p.ranges.clear();
      p.intensities.clear();

      // Interpolate the range readings from the rays
      for (i = 0; i<rangeCount; i++)
      {
        b = (double) i * (rayCount - 1) / (rangeCount - 1);
        ja = (int) floor(b);
        jb = std::min(ja + 1, rayCount - 1);
        b = b - floor(b);

        assert(ja >= 0 && ja < rayCount);
        assert(jb >= 0 && jb < rayCount);

        ra = std::min(this->laser->GetLaserShape()->GetRange(ja) , maxRange-minRange); // length of ray
        rb = std::min(this->laser->GetLaserShape()->GetRange(jb) , maxRange-minRange); // length of ray

        // Range is linear interpolation if values are close,
        // and min if they are very different
        //if (fabs(ra - rb) < 0.10)
          r = (1 - b) * ra + b * rb;
        //else r = std::min(ra, rb);

        // Intensity is averaged
        intensity = 0.5*( this->laser->GetLaserShape()->GetRetro(ja)
                        + this->laser->GetLaserShape()->GetRetro(jb));

        /***************************************************************/
        /*                                                             */
        /*  point scan from laser                                      */
        /*                                                             */
        /***************************************************************/
        //p.ranges.push_back(std::min(r + minRange + this->GaussianKernel(0,this->gaussian_noise_), maxRange));
        p.ranges.push_back(r);
        //p.intensities.push_back(std::max(this->hokuyo_min_intensity_,intensity + this->GaussianKernel(0,this->gaussian_noise_)));
      }

      this->laser->SetActive(true);

      // send data out via ros message
      this->pub.publish(p);


    }




    void ROSCallback(const geometry_msgs::Point::ConstPtr& msg) 
    {
      ROS_INFO("subscriber got something0: [%f]", msg->x);
      ROS_INFO("subscriber got something1: [%f]", msg->y);
  
        this->leftWheelJoint->SetForce(0, double(msg->x));
        this->rightWheelJoint->SetForce(0, double(msg->y));


    }

    // Pointer to the model
    private: physics::ModelPtr model;

    // Pointer to the update event connection
    private: event::ConnectionPtr updateConnection;

    //Added
    private: physics::JointPtr leftWheelJoint;
    private: physics::JointPtr rightWheelJoint;
    private: sensors::RaySensorPtr laser;
    private: double gain;

    // ROS Nodehandle
    private: ros::NodeHandle* node;

    // ROS Subscriber
    ros::Subscriber sub;
    ros::Publisher pub;
  };

  // Register this plugin with the simulator
  GZ_REGISTER_MODEL_PLUGIN(ROSModelPlugin)
}
