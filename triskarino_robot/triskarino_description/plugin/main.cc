#ifndef _OMNI_WHEEL_CONTROL_
#define _OMNI_WHEEL_CONTROL_


#include <algorithm>
#include <assert.h>

#include <ignition/math/Angle.hh>
#include <ignition/math/Pose3.hh>
#include <ignition/math/Quaternion.hh>
#include <ignition/math/Vector3.hh>
#include <sdf/sdf.hh>

#include <map>

// Gazebo
#include <gazebo/common/common.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo_plugins/gazebo_ros_utils.h>
#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>
// ROS
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/Pose2D.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/JointState.h>
#include <ros/subscribe_options.h>

// Custom Callback Queue
#include <ros/callback_queue.h>
#include <ros/advertise_options.h>

// Boost
#include <boost/thread.hpp>
#include <boost/bind.hpp>



namespace gazebo
{

    
    class OmniWheelControl : public ModelPlugin {
    
    //Angular velocity of the 3 wheels
    struct WheelVelocities{
        float vLeft;
        float vBack;
        float vRight;
    };

    //Robot twist: linear speed on the x,y axis and angular velocity on the z axis in the robot frame
    struct RobotTwist{
        float vX;
        float vY;
        float wZ;
    };

    //Robot pose: position of the robot on the x and y axis of the world frame, and angle theta between the robot and the world frame
    struct RobotPose{
        float x;
        float y;
        float theta;
    };

    enum OdomSource
    {
        ENCODER = 0,
        WORLD = 1,
    };

    private:

      physics::JointPtr backJoint;

      physics::JointPtr leftJoint;

      // Pointer to the model
      physics::ModelPtr model;

      physics::JointPtr rightJoint;

      // Pointer to the update event connection
      event::ConnectionPtr updateConnection;

      physics::WorldPtr world;
      // measured in meters
      float wheelRadius;
      // measured in meters
      float wheelToCenterDist;

      common::PID pid;

      /// \brief A node use for ROS transport
      std::unique_ptr<ros::NodeHandle> rosNode;

      /// \brief A ROS subscriber
      ros::Subscriber rosVelSubscriber;

      ros::Publisher rosOdomPublisher;

      ros::Publisher rosJointStatePublisher;      

      /// \brief A ROS callbackqueue that helps process messages
      ros::CallbackQueue rosQueue;

      /// \brief A thread the keeps running the rosQueue
      std::thread rosQueueThread;
      // m/s
      float maxLinearSpeed;
      // rad/s
      float maxAngularSpeed;
      //topic where velocities are published
      std::string velTopicName;
      //topic where odometry is published
      std::string odomTopicName;
      //odometry frame
      std::string odometryFrame;
      //robot base frame
      std::string robotBaseFrame;
      //Publish odom tf
      bool publishOdomTf_;
      //Publish Tf
      bool publishWheelTf_;
      //Publish Wheel Joint State
      bool publishWheelJointState_;
      //If true expecting twist messages to be stamped
      bool isTwistStamped;
      //Update Rate
      double updateRate;
    
      double updatePeriod;
      common::Time lastUpdateTime;

      OdomSource odomSource;
      ros::Time lastOdomUpdate;
      
      //Requested Wheel Velocities
      OmniWheelControl::WheelVelocities requestedWheelVelocities;
      //Curent Wheel Velocities
      OmniWheelControl::WheelVelocities currentWheelVelocities;
      //Requested Twist
      OmniWheelControl::RobotTwist requestedTwist;
      //Current Robot Twist
      OmniWheelControl::RobotTwist currentTwist;
      //Robot Pose
      OmniWheelControl::RobotPose currentRobotPose;
      //Alive Bool 
      bool alive;
      //Transform Broadcaster
      boost::shared_ptr<tf::TransformBroadcaster> transformBroadcaster;

      nav_msgs::Odometry odomMsg;

      sensor_msgs::JointState jointStateMsg;

      boost::mutex lock;

      ignition::math::Vector3d starting_fdir = ignition::math::Vector3d::Zero;

          /// \brief Constructor
    public: OmniWheelControl() {}

    
    /// \brief The load function is called by Gazebo when the plugin is
    /// inserted into simulation
    /// \param[in] _model A pointer to the model that this plugin is
    /// attached to.
    /// \param[in] _sdf A pointer to the plugin's SDF element.
    public: virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
    {
      float Kp;
      float Ki;
      float Kd;
      std::string leftJointName;
      std::string rightJointName;
      std::string backJointName;
      // Initialize ros, if it has not already been initialized.
      if (!ros::isInitialized())
      {
        int argc = 0;
        char **argv = NULL;
        ros::init(argc, argv, "gazebo_client",ros::init_options::NoSigintHandler);
      }

      // Create our ROS node. This acts in a similar manner to
      // the Gazebo node
      this->rosNode.reset(new ros::NodeHandle("gazebo_client"));

      ROS_DEBUG_NAMED("LOADING PLUGIN", "Into Plugin Load Function");

      if (_sdf->HasElement("wheelRadius"))
        this->wheelRadius = _sdf->Get<double>("wheelRadius");
      else{
        this->wheelRadius = 0.03;
        ROS_DEBUG_NAMED("wheelRadius", "wheelRadius parameter missing, defaults to 0.03");
      }

      if (_sdf->HasElement("wheelToCenterDist"))
        this->wheelToCenterDist = _sdf->Get<double>("wheelToCenterDist");
      else{
        this->wheelToCenterDist = 0.165;
        ROS_DEBUG_NAMED("wheelToCenterDist", "wheelToCenterDist parameter missing, defaults to 0.165");
      }

      if (_sdf->HasElement("Kp"))
        Kp = _sdf->Get<double>("Kp");
       else{
        Kp = 0.01;
        ROS_DEBUG_NAMED("Kp", "Kp parameter missing, defaults to 0.01");
       }

      if (_sdf->HasElement("Ki"))
        Ki = _sdf->Get<double>("Ki");
      else{
        Ki = 0.0;
        ROS_DEBUG_NAMED("Ki", "Ki parameter missing, defaults to 0.0");
       }
      
      if (_sdf->HasElement("Kd"))
        Kd = _sdf->Get<double>("Kd");
      else{
        Kd = 0.0;
        ROS_DEBUG_NAMED("Kd", "Kd parameter missing, defaults to 0.0");
       }

      if (_sdf->HasElement("leftJoint"))
        leftJointName = _sdf->Get<std::string>("leftJoint");
      else{
        leftJointName = "left_joint";
        ROS_DEBUG_NAMED("leftJoint", "leftJoint parameter missing, defaults to \"left_joint\"");
       }
    
      if (_sdf->HasElement("rightJoint"))
        rightJointName = _sdf->Get<std::string>("rightJoint");
      else{
        leftJointName = "right_joint";
        ROS_DEBUG_NAMED("rightJoint", "rightJoint parameter missing, defaults to \"right_joint\"");
       }

      if (_sdf->HasElement("backJoint"))
        backJointName = _sdf->Get<std::string>("backJoint");
      else{
        leftJointName = "back_joint";
        ROS_DEBUG_NAMED("backJoint", "backJoint parameter missing, defaults to \"back_joint\"");
       }

      if (_sdf->HasElement("maxLinearSpeed"))
        this->maxLinearSpeed = _sdf->Get<float>("maxLinearSpeed");
      else{
        this->maxLinearSpeed = 1;
        ROS_DEBUG_NAMED("maxLinearSpeed", "maxLinearSpeed parameter missing, defaults to 1");
      }

      if (_sdf->HasElement("maxAngularSpeed"))
        this->maxAngularSpeed = _sdf->Get<float>("maxAngularSpeed");
      else{
        this->maxAngularSpeed = 3.14;
        ROS_DEBUG_NAMED("maxAngularSpeed", "maxAngularSpeed parameter missing, defaults to 3.14");
      }

      if (_sdf->HasElement("velTopicName"))
        this->velTopicName = _sdf->Get<std::string>("velTopicName");
      else{
        this->velTopicName = "cmd_vel";
        ROS_DEBUG_NAMED("velTopicName", "velTopicName parameter missing, defaults to \"cmd_vel\"");
      }
      
      if (_sdf->HasElement("odomTopicName"))
        this->odomTopicName = _sdf->Get<std::string>("odomTopicName");
      else{
        this->odomTopicName = "odom";
        ROS_DEBUG_NAMED("odomTopicName", "odomTopicName parameter missing, defaults to \"odom\"");
      }
      
      if (_sdf->HasElement("odometryFrame"))
        this->odometryFrame = _sdf->Get<std::string>("odometryFrame");
      else{
        this->velTopicName = "odom";
        ROS_DEBUG_NAMED("odometryFrame", "odometryFrame parameter missing, defaults to \"odometryFrame\"");
      }

      if (_sdf->HasElement("robotBaseFrame"))
        this->robotBaseFrame = _sdf->Get<std::string>("robotBaseFrame");
      else{
        this->robotBaseFrame = "base_footprint";
        ROS_DEBUG_NAMED("robotBaseFrame", "robotBaseFrame parameter missing, defaults to \"base_footprint\"");
      }

      if (_sdf->HasElement("publishOdomTf"))
        this->publishOdomTf_ = _sdf->Get<bool>("publishOdomTf");
      else{
        this->publishOdomTf_ = false;
        ROS_DEBUG_NAMED("publishOdomTf", "publishOdomTf parameter missing, defaults to false");
      }

      if (_sdf->HasElement("publishWheelJointState"))
        this->publishWheelJointState_ = _sdf->Get<bool>("publishWheelJointState");
      else{
        this->publishWheelJointState_ = false;
        ROS_DEBUG_NAMED("publishWheelJointState", "publishWheelJointState parameter missing, defaults to false");
      }
      
      if (_sdf->HasElement("publishWheelTf"))
        this->publishWheelTf_ = _sdf->Get<bool>("publishWheelTf");
      else{
        this->publishWheelTf_ = false;
        ROS_DEBUG_NAMED("publishWheelTf", "publishWheelTf parameter missing, defaults to false");
      }

      if (_sdf->HasElement("updateRate"))
        this->updateRate = _sdf->Get<double>("updateRate");
      else{
        this->updateRate = 100.0;
        ROS_DEBUG_NAMED("updateRate", "updateRate parameter missing, defaults to updateRate");
      }

      if (_sdf->HasElement("updateRate"))
        this->updateRate = _sdf->Get<double>("updateRate");
      else{
        this->updateRate = 100.0;
        ROS_DEBUG_NAMED("updateRate", "updateRate parameter missing, defaults to updateRate");
      }

      if (_sdf->HasElement("odomSource"))
        this->odomSource = _sdf->Get<std::string>("odomSource") == "encoder" ? OdomSource(ENCODER) : OdomSource(WORLD);
      else{
        this->odomSource = OdomSource(ENCODER);
        ROS_DEBUG_NAMED("odomSource", "odomSource parameter missing, defaults to Encoder");
      }


      if (_sdf->HasElement("isTwistStamped"))
        this->isTwistStamped = _sdf->Get<bool>("isTwistStamped");
      else{
        this->isTwistStamped = false;
        ROS_DEBUG_NAMED("isTwistStamped", "isTwistStamped parameter missing, defaults to false");
      }
      

      this->model = _model;
      this->world = this->model->GetWorld();
      this->leftJoint = this->model->GetJoint(leftJointName);
      this->rightJoint = this->model->GetJoint(rightJointName);
      this->backJoint = this->model->GetJoint(backJointName);

      if(this->leftJoint == NULL)
        gzdbg << "Left Joint name not found name was " + leftJointName << std::endl;
      if(this->rightJoint == NULL)
        gzdbg << "Right Joint name not found name was " + rightJointName<< std::endl;
      if(this->backJoint == NULL)
        gzdbg << "Back Joint name not found name was " + backJointName << std::endl;
      
      this->pid = common::PID(Kp, Ki, Kd);

      /*this->model->GetJointController()->SetVelocityPID(
        this->leftJoint->GetScopedName(), this->pid);
    
      this->model->GetJointController()->SetVelocityPID(
        this->backJoint->GetScopedName(), this->pid);
    
      this->model->GetJointController()->SetVelocityPID(
        this->rightJoint->GetScopedName(), this->pid);*/
      
      if ( this->updateRate > 0.0 ) this->updatePeriod = 1.0 / this->updateRate;
      else this->updatePeriod = 0.0;
      #if GAZEBO_MAJOR_VERSION >= 8
        this->lastUpdateTime = this->model->GetWorld()->SimTime();
      #else
        this->lastUpdateTime = this->model->GetWorld()->GetSimTime();
      #endif

      this->currentWheelVelocities.vLeft = 0.0;
      this->currentWheelVelocities.vRight = 0.0;
      this->currentWheelVelocities.vBack = 0.0;
      this->requestedTwist.vX = 0.0;
      this->requestedTwist.vY = 0.0;
      this->requestedTwist.wZ = 0.0;
      this->requestedWheelVelocities.vLeft = 0.0;
      this->requestedWheelVelocities.vRight = 0.0;
      this->requestedWheelVelocities.vBack = 0.0;
      this->currentTwist.vX = 0.0;
      this->currentTwist.vY = 0.0;
      this->currentTwist.wZ = 0.0;
      this->currentRobotPose.x = 0.0;
      this->currentRobotPose.y = 0.0;
      this->currentRobotPose.theta = 0.0;
      this->alive = true;

      ros::SubscribeOptions so =
        ros::SubscribeOptions::create<geometry_msgs::Twist>(
          this->velTopicName,
          1, 
          boost::bind(&OmniWheelControl::OnRosMsg, this, _1),
          ros::VoidPtr(), &this->rosQueue);;

      if (this->isTwistStamped == true){
        so =
        ros::SubscribeOptions::create<geometry_msgs::TwistStamped>(
          this->velTopicName,
          1, 
          boost::bind(&OmniWheelControl::OnRosMsgStamped, this, _1),
          ros::VoidPtr(), &this->rosQueue);
      }
     
      this->rosVelSubscriber = this->rosNode->subscribe(so);

      // Create a named topic, and subscribe to it.
      
      ROS_INFO_NAMED("omni_wheel", "Subscribed to %s", velTopicName.c_str());

      // Spin up the queue helper thread.
      this->rosQueueThread = std::thread(std::bind(&OmniWheelControl::QueueThread, this));

      if(this->publishWheelJointState_){
        this->rosJointStatePublisher = this->rosNode->advertise<sensor_msgs::JointState>("joint_states", 1000);
        ROS_INFO_NAMED("omni_wheel", "Advertising joint_states");
      }

      this->transformBroadcaster = boost::shared_ptr<tf::TransformBroadcaster>(new tf::TransformBroadcaster());
    
      if(this->publishOdomTf_){
        this->rosOdomPublisher = this->rosNode->advertise<nav_msgs::Odometry>(this->odomTopicName,1);
        ROS_INFO_NAMED("omni_wheel", "Advertising odom on %s ", this->odomTopicName.c_str());

      }

      // listen to the update event (broadcast every simulation iteration)
      this->updateConnection =  
        event::Events::ConnectWorldUpdateBegin ( boost::bind ( &OmniWheelControl::UpdatePID, this ) );
    }

    private: void inverseKinematics() {
        boost::mutex::scoped_lock scoped_lock ( this->lock );
        this->requestedWheelVelocities.vBack = (-this->wheelToCenterDist * this->requestedTwist.wZ + this->requestedTwist.vX)/this->wheelRadius;
        this->requestedWheelVelocities.vLeft = (-this->wheelToCenterDist * this->requestedTwist.wZ -0.5f*this->requestedTwist.vX - 0.866025404f*this->requestedTwist.vY)/this->wheelRadius;
        this->requestedWheelVelocities.vRight = (-this->wheelToCenterDist * this->requestedTwist.wZ -0.5f*this->requestedTwist.vX + 0.866025404f*this->requestedTwist.vY)/this->wheelRadius;
    }

    private: void forwardKinematics(){
        this->currentTwist.wZ = -this->wheelRadius*(this->currentWheelVelocities.vRight + this->currentWheelVelocities.vBack + this->currentWheelVelocities.vLeft)/(3.0*this->wheelToCenterDist);
        this->currentTwist.vX = this->wheelRadius*(+2.0*this->currentWheelVelocities.vBack - this->currentWheelVelocities.vLeft - this->currentWheelVelocities.vRight)/3.0;
        this->currentTwist.vY = sqrt(3)*this->wheelRadius*(this->currentWheelVelocities.vRight - this->currentWheelVelocities.vLeft)/3.0;
    }

    private: void UpdatePID(){
          #if GAZEBO_MAJOR_VERSION >= 8
        common::Time current_time = model->GetWorld()->SimTime();
        #else
        common::Time current_time = model->GetWorld()->GetSimTime();
        #endif
        double seconds_since_last_update = ( current_time - this->lastUpdateTime ).Double();

        if ( seconds_since_last_update > this->updatePeriod ) {
            if (this->publishOdomTf_) publishOdometry ( seconds_since_last_update );
            if (this->publishWheelTf_) publishAllWheelsTf();
            if (this->publishWheelJointState_) publishAllWheelsJointState();
            this->moveRobot();
            this->lastUpdateTime += common::Time ( this->updatePeriod );
        }
    }

    private: void publishOdometry(double step_time){
        ros::Time current_time = ros::Time::now();
        if (this->odomSource == ENCODER){
            updateOdometryEncoder((current_time - this->lastOdomUpdate).toSec());
        }
        else{
            updateOdometryWorld();
        }
        //Now Current twist and pose of the robot are updated, use them to make odometry
        geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(this->currentRobotPose.theta);
        geometry_msgs::TransformStamped odom_trans;
        odom_trans.header.stamp = current_time;
        odom_trans.header.frame_id = this->odometryFrame;
        odom_trans.child_frame_id = this->robotBaseFrame;
        odom_trans.transform.translation.x = this->currentRobotPose.x;
        odom_trans.transform.translation.y = this->currentRobotPose.y;
        odom_trans.transform.translation.z = 0.0;
        odom_trans.transform.rotation = odom_quat;
        this->transformBroadcaster->sendTransform(odom_trans);
        this->odomMsg.header.stamp = current_time;
        this->odomMsg.header.frame_id = this->odometryFrame;
        this->odomMsg.pose.pose.position.x = this->currentRobotPose.x;
        this->odomMsg.pose.pose.position.y = this->currentRobotPose.y;
        this->odomMsg.pose.pose.position.z = 0.0;
        this->odomMsg.pose.pose.orientation = odom_quat;
        this->odomMsg.child_frame_id = this->robotBaseFrame;
        this->odomMsg.twist.twist.linear.x = this->currentTwist.vX;
        this->odomMsg.twist.twist.linear.y = this->currentTwist.vY;
        this->odomMsg.twist.twist.angular.z = this->currentTwist.wZ;
        this->rosOdomPublisher.publish(this->odomMsg);
        this->lastOdomUpdate = current_time;

    }


    private: void updateOdometryWorld(){
        #if GAZEBO_MAJOR_VERSION >= 8
            ignition::math::Pose3d pose = this->model->WorldPose();
            ignition::math::Vector3d linear = this->model->WorldLinearVel();
            double angular = this->model->WorldAngularVel().Z();
        #else
            ignition::math::Pose3d pose = this->model->GetWorldPose().Ign();
            ignition::math::Vector3d linear = this->model->GetWorldLinearVel().Ign();
            double angular = this->model->GetWorldAngularVel().Ign().Z();
        #endif
        //Velocities are in the world frame, we have to convert it to the robot frame
        float yaw = pose.Rot().Yaw();
        this->currentTwist.vX = cosf(yaw) * linear.X() + sinf(yaw) * linear.Y();
        this->currentTwist.vY = cosf(yaw) * linear.Y() - sinf(yaw) * linear.X();
        this->currentTwist.wZ = angular;
        this->currentRobotPose.x = pose.X();
        this->currentRobotPose.y = pose.Y();
        //TODO: Check if this is correct
        this->currentRobotPose.theta = yaw;
    }

    private: void updateOdometryEncoder(double deltaTime){
        getWheelVelocitiesFromJoints();
        forwardKinematics();
        double deltaTheta = this->currentTwist.wZ * deltaTime;
        double deltaX = (this->currentTwist.vX * cos(this->currentRobotPose.theta) - this->currentTwist.vY * sin(this->currentRobotPose.theta)) * deltaTime;
        double deltaY = (this->currentTwist.vX * sin(this->currentRobotPose.theta) + this->currentTwist.vY * cos(this->currentRobotPose.theta)) * deltaTime;
        this->currentRobotPose.x = this->currentRobotPose.x + deltaX;
        this->currentRobotPose.y = this->currentRobotPose.y + deltaY;
        this->currentRobotPose.theta = this->currentRobotPose.theta + deltaTheta;
        this->currentRobotPose.theta = fmod(this->currentRobotPose.theta,(float)M_PI);
    }

    private: void getWheelVelocitiesFromJoints(){
        this->currentWheelVelocities.vBack = this->backJoint->GetVelocity(0);
        this->currentWheelVelocities.vLeft = this->leftJoint->GetVelocity(0);
        this->currentWheelVelocities.vRight = this->rightJoint->GetVelocity(0);
    }

    /// \brief Handle an incoming message from ROS
    /// \param[in] _msg A float value that is used to set the velocity 
    public: void OnRosMsg(const geometry_msgs::TwistConstPtr twistMsg)
    {
        boost::mutex::scoped_lock scoped_lock ( this->lock );
        requestedTwist.vX = twistMsg->linear.x * this->maxLinearSpeed;
        requestedTwist.vY = twistMsg->linear.y * this->maxLinearSpeed;
        requestedTwist.wZ = twistMsg->angular.z * this->maxAngularSpeed;
    }

     /// \brief Handle an incoming message from ROS
    /// \param[in] _msg A float value that is used to set the velocity 
    public: void OnRosMsgStamped(const geometry_msgs::TwistStampedConstPtr twistMsg)
    {
        boost::mutex::scoped_lock scoped_lock ( this->lock );
        requestedTwist.vX = twistMsg->twist.linear.x * this->maxLinearSpeed;
        requestedTwist.vY = twistMsg->twist.linear.y * this->maxLinearSpeed;
        requestedTwist.wZ = twistMsg->twist.angular.z * this->maxAngularSpeed;
    }


    /// \brief ROS helper function that processes messages
    private: void QueueThread()
    {
        static const double timeout = 0.01;
        while (this->rosNode->ok())
        {
            this->rosQueue.callAvailable(ros::WallDuration(timeout));
        }
    }
    private: void moveRobot(){
        inverseKinematics();
        /*this->model->GetJointController()->SetVelocityTarget(
            this->leftJoint->GetScopedName(), requestedWheelVelocities.vLeft);
        this->model->GetJointController()->SetVelocityTarget(
            this->backJoint->GetScopedName(), requestedWheelVelocities.vBack);
        this->model->GetJointController()->SetVelocityTarget(
            this->rightJoint->GetScopedName(), requestedWheelVelocities.vRight);*/
        this->rightJoint->SetVelocity(0, requestedWheelVelocities.vRight);
        this->backJoint->SetVelocity(0, requestedWheelVelocities.vBack);
        this->leftJoint->SetVelocity(0, requestedWheelVelocities.vLeft);     

    }

    private: void publishAllWheelsTf(){
        ros::Time current_time = ros::Time::now();
        this->publishWheelTf(this->leftJoint,current_time);
        this->publishWheelTf(this->rightJoint,current_time);
        this->publishWheelTf(this->backJoint,current_time);

    }

    private: void publishWheelTf(physics::JointPtr wheel, ros::Time current_time){
        std::string wheel_frame = wheel->GetChild()->GetName ();
        std::string wheel_parent_frame = wheel->GetParent()->GetName();
        #if GAZEBO_MAJOR_VERSION >= 8
            ignition::math::Pose3d poseWheel = wheel->GetChild()->RelativePose();
        #else
            ignition::math::Pose3d poseWheel = wheel->GetChild()->GetRelativePose().Ign();
        #endif
        tf::Quaternion qt ( poseWheel.Rot().X(), poseWheel.Rot().Y(), poseWheel.Rot().Z(), poseWheel.Rot().W() );
        tf::Vector3 vt ( poseWheel.Pos().X(), poseWheel.Pos().Y(), poseWheel.Pos().Z() );
        tf::Transform tfWheel ( qt, vt );
        this->transformBroadcaster->sendTransform (
            tf::StampedTransform ( tfWheel, current_time, wheel_parent_frame, wheel_frame ) );
    }

    private: void publishAllWheelsJointState(){
        ros::Time current_time = ros::Time::now();
        jointStateMsg.header.stamp = current_time;
        jointStateMsg.name.resize (3);
        jointStateMsg.position.resize (3);
        fillWheelJointStateMsg(this->leftJoint, current_time, 0);
        fillWheelJointStateMsg(this->backJoint, current_time, 1);
        fillWheelJointStateMsg(this->backJoint, current_time, 2);
        rosJointStatePublisher.publish ( this->jointStateMsg );

    }

    private: void fillWheelJointStateMsg(physics::JointPtr wheel, ros::Time current_time, int index){
        #if GAZEBO_MAJOR_VERSION >= 8
            double position = wheel->Position ( 0 );
        #else
            double position = wheel->GetAngle ( 0 ).Radian();
        #endif
        jointStateMsg.name[index] = wheel->GetName();
        jointStateMsg.position[index] = position;
    }
  };



  // Tell Gazebo about this plugin, so that Gazebo can call Load on this plugin.
  GZ_REGISTER_MODEL_PLUGIN(OmniWheelControl)
   
}
#endif
