/**
 * @file /src/qnode.cpp
 *
 * @brief Ros communication central!
 *
 * @date February 2011
 **/


/*****************************************************************************
** Includes
*****************************************************************************/

#include <ros/ros.h>
#include <ros/network.h>
#include <string>
#include <std_msgs/String.h>
#include <sstream>
#include "../include/learninggui/qnode.hpp"
#include <sensor_msgs/Joy.h>
#include<string.h>

/*****************************************************************************
** Namespaces
********************
*********************************************************/

// at full joystick depression you'll go this fast
double max_speed = 0.100; // m/second
double max_turn = 60.0*M_PI/180.0; // rad/second
// should we continuously send commands?
//bool always_command = false;

//QNode* tbk;
//int kfd = 0;
//struct termios cooked, raw;
//bool done;

namespace learninggui {
using namespace std;
/*****************************************************************************
** Implementation
*****************************************************************************/
QNode::QNode(){
     qDebug()<<"am in qnode constructor1"<<endl;

      max_tv_ = max_speed;
      max_rv_ = max_turn;
     // init();
}

QNode::QNode(int argc, char** argv ) :
    init_argc(argc),
    init_argv(argv)
    {


}

QNode::~QNode() {
    if(ros::isStarted()) {
      ros::shutdown(); // explicitly needed since we use ros::start();
      ros::waitForShutdown();
    }
    wait();
}
bool QNode::init() {
  ros::init(init_argc,init_argv,"learninggui");
   qDebug()<<"am in init"<<endl;
 if ( ! ros::master::check() ) {
      return false;
  }
  ros::start(); // explicitly needed since our nodehandle is going out of scope.

  ros::NodeHandle n_;
   pub_ = n_.advertise<geometry_msgs::Twist>("/RosAria/cmd_vel",1); //TBK_Node publishes RosAria/cmd_vel topic and RosAria node subscribes to it

//ros::spin();
   start();
    return true;
}

void QNode::keyPressed(int keyId)
{

    QMutexLocker lock(&mutex);
    isKeyPressed = true;
    this->keyId = keyId;
     qDebug()<<"here in keypressed key id is "<<keyId;
}


void QNode::run()
{
    ros::Rate loop_rate(1);
    //int count = 0;
    //ros::spin();
    ros::spinOnce();
    int scalingfactor = 1;
    double x = 0.1;
    while(ros::ok() /*&& pub_*/)
    {
       if(isKeyPressed)
        {
            // do something
            mutex.lock();
            isKeyPressed = false;
            mutex.unlock();
            switch(keyId)
            {
            case SPACEB:
               cmdvel.linear.x=0.0;
               cmdvel.angular.z=0.0;
                break;
            case DOWN:
                cmdvel.linear.x = -x;
                cmdvel.angular.z = 0.0;
                break;
            case UP:
                cmdvel.linear.x = x;
                cmdvel.angular.z = 0.0;
                break;
            case RIGHTB:
                cmdvel.linear.x = 0.0;//0.2
                cmdvel.angular.z = -0.3;//doesn't always work
                break;
            case LEFTB:
                cmdvel.linear.x = 0.0;//0.1
                cmdvel.angular.z = 0.3;
                break;
            case SHIFTB:
                scalingfactor++;
                if(scalingfactor == 7)
                {
                    scalingfactor =  6;
                }
                else if(scalingfactor <= 0)
                {
                    scalingfactor = 1;
                }
                x = 0.1 * scalingfactor;
                if(cmdvel.linear.x<0)
                {
                    cmdvel.linear.x = -x;
                    cmdvel.angular.z = 0.0;
                }
                else if(cmdvel.linear.x>0)
                {
                    cmdvel.linear.x = x;
                    cmdvel.angular.z = 0.0;
                }
                break;
            case CTRLB:
                scalingfactor--;
                if(scalingfactor == 7)
                {
                    scalingfactor =  6;
                }
                else if(scalingfactor <= 0)
                {
                    scalingfactor = 1;
                }
                x = 0.1 * scalingfactor;
                if(cmdvel.linear.x<0)
                {
                    cmdvel.linear.x = -x;
                    cmdvel.angular.z = 0.0;
                }
                else if(cmdvel.linear.x>0)
                {
                    cmdvel.linear.x = x;
                    cmdvel.angular.z = 0.0;
                }
                break;
            default:
                qDebug()<<"unknown key";
            }
           QString m= QString::number(cmdvel.linear.x);
           QString z= QString::number(cmdvel.angular.z);

           log(Info,std::string("Linear velocity:  "+m.toStdString()+"\nAngular velocity:  "
                                +z.toStdString()));

           // speed
            pub_.publish(cmdvel);


            loop_rate.sleep();
        }
    }
}

double QNode::speed(){
    double kk=cmdvel.linear.x;
    return kk;
}

double QNode::speed2(){
    double kk=cmdvel.linear.z;
    return kk;
}

void QNode::log( const LogLevel &level, const std::string &msg) {
    logging_model.insertRows(logging_model.rowCount(),1);
    std::stringstream logging_model_msg;
    switch ( level ) {
        case(Debug) : {
                ROS_DEBUG_STREAM(msg);
                logging_model_msg << "[DEBUG] [" << ros::Time::now() << "]: " << msg;
                break;
        }
        case(Info) : {
                logging_model.removeRows(1,1);
                ROS_INFO_STREAM(msg);
                logging_model_msg << /*"[INFO] [" << ros::Time::now() << "]: " <<*/ msg;
                break;
        }
        case(Warn) : {
                ROS_WARN_STREAM(msg);
                logging_model_msg << "[INFO] [" << ros::Time::now() << "]: " << msg;
                break;
        }
        case(Error) : {
                ROS_ERROR_STREAM(msg);
                logging_model_msg << "[ERROR] [" << ros::Time::now() << "]: " << msg;
                break;
        }
        case(Fatal) : {
                ROS_FATAL_STREAM(msg);
                logging_model_msg << "[FATAL] [" << ros::Time::now() << "]: " << msg;
                break;
        }
    }
    QVariant new_row(QString(logging_model_msg.str().c_str()));
    logging_model.setData(logging_model.index(logging_model.rowCount()-1),new_row);
    Q_EMIT loggingUpdated(); // used to readjust the scrollbar
}


void QNode::stopRobot()
{
  cmdvel.linear.x = cmdvel.angular.z = 0.0;
  pub_.publish(cmdvel);
}
}  // namespace learninggui
