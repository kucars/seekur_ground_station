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
#include "../include/learningjoint/qnode.hpp"
#include <sensor_msgs/JointState.h>
#include<string.h>

/*****************************************************************************
** Namespaces
********************
*********************************************************/

// at full joystick depression you'll go this fast
double max_speed = 0.100; // m/second
double max_turn = 60.0*M_PI/180.0; // rad/second
// should we continuously send commands?
bool always_command = false;

//QNode* tbk;
int kfd = 0;
struct termios cooked, raw;
bool done;

namespace learningjoint {
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

   // init();
    }

QNode::~QNode() {
    if(ros::isStarted()) {
      ros::shutdown(); // explicitly needed since we use ros::start();
      ros::waitForShutdown();
    }
    wait();
}
bool QNode::init() {
  ros::init(init_argc,init_argv,"learningjoint");
   qDebug()<<"am in init"<<endl;
 if ( ! ros::master::check() ) {
      return false;
  }
  ros::start(); // explicitly needed since our nodehandle is going out of scope.

  ros::NodeHandle n_;
  // pub_ = n_.advertise<geometry_msgs::Twist>("/RosAria/cmd_vel",1); //TBK_Node publishes RosAria/cmd_vel topic and RosAria node subscribes to it
  pub_ = n_.advertise<sensor_msgs::JointState>("/joint_states",1);
   // joy_sub_ = n_.subscribe<joy::Joy>("/joy", 10, &QNode::joyCallback, this);

   qDebug()<<"am done with pub snd add"<<endl;

  chatter_publisher = n_.advertise<std_msgs::String>("chatter", 1000);
//    pub_ = n_.advertise<geometry_msgs::Twist>("/cmd_vel",1); //TBK_Node publishes RosAria/cmd_vel topic and RosAria node subscribes to it
  // joy_sub_ = n_.subscribe<sensor_msgs::Joy>("joy", 10, &QNode::joyCallback, this);
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


    ros::Rate loop_rate(10);
    int selectedjoint;
    // double i[5]={0,-40,-90,-40,0};
    i[0]=0;
    i[1]=-40;
    i[2]=-90;
    i[3]=-40;
    i[4]= 0;
    double k = 0;
    QString grip_status;

    while(ros::ok() /*&& pub_*/)
    {
        QString name ;
        if(isKeyPressed)
        {
            //QString name ;
            double jointPose = 0;
            if(joint == QString("j1"))
            {
                name = "j1";
                selectedjoint=0;
                jointPose = i[0];
            }
            else if(joint == QString("j2"))
            {
                qDebug()<<" Joint 2"<<jointPose;
                name = "j2";
                selectedjoint=1;
                jointPose =i[1];

            }
            else if(joint == QString("j3"))
            {
                name = "j3";
                selectedjoint=2;
                jointPose =i[2];
                qDebug()<<" Joint 3"<<jointPose;

            }
            else if(joint == QString("j4"))
            {
                name = "j4";
                selectedjoint=3;
                jointPose =i[3];
                qDebug()<<" Joint 3"<<jointPose;

            }
            else if(joint == QString("j5"))
            {
                name = "j5";
                selectedjoint=4;
                jointPose =i[4];
                qDebug()<<" Joint 3"<<jointPose;

            }
            else if(joint == QString("g"))
            {
                name = "g";

                qDebug()<<" Gripper"<<jointPose;
            }
            else if(joint == QString("grasp"))
            {
                name = "grasp";

            }

            qDebug()<<"Moving Joint"<<name<<" and joint is:"<<joint;
            // do something
            mutex.lock();
            isKeyPressed = false;
            mutex.unlock();
            switch(keyId)
            {
            case UP:
                if(name == QString("g"))
                {
                    grip_status = "open";
                    j.name.push_back(name.toStdString());
                    j.position.push_back(0);
                    pub_.publish(j);
                }

                else
                {
                j.name.push_back(name.toStdString());
                i[selectedjoint] = jointPose+10;
                j.position.push_back(i[selectedjoint]);
                pub_.publish(j);
                }
                break;
            case DOWN:
                if(name == QString("g"))
                {
                    grip_status = "closed";
                    j.name.push_back(name.toStdString());
                    j.position.push_back(1);
                    pub_.publish(j);
                }
                else if(name == QString("grasp"))
                {
                    j.name.push_back(name.toStdString());
                    j.position.push_back(1);
                    pub_.publish(j);
                }
                else
                {
                j.name.push_back(name.toStdString());
                i[selectedjoint] = jointPose-10;
                j.position.push_back(i[selectedjoint]);
                pub_.publish(j);
                }

                break;

            }

            QString stringjoint = "joint 1 \t joint 2 \t joint 3 \t joint 4 \t joint 5 \t Gripper\n";
            QString string1 = "";

            for (int k=0;k<5;k++)
             {
                 string1 = string1+QString::number(i[k])+"\t";//qt visualizers seen view //radio checks//regidtry q settings
             }

           // log(Info,std::string("Linear velocity:  "+j1.toStdString()));
            string1 = string1 + grip_status;
            log(Info,std::string(stringjoint.toStdString()+string1.toStdString()));

            qDebug()<<"Size of j:"<<j.name.size();
        }
        std_msgs::String msg;
        chatter_publisher.publish(msg);

        j.name.clear();
        j.position.clear();
        ros::spinOnce();
        loop_rate.sleep();
    }
}

double* QNode::jointpostion(){
    double* kk= new double[5];
   // kk[3]=i[3];
    for (int h=0;h<5;h++)
    {
        kk[h]=i[h];
        qDebug()<<"IN joint position:  ";
        qDebug()<<kk[h];
   }
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
                logging_model_msg <</* "[INFO] [" << ros::Time::now() << "]: " <<*/msg;
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

void QNode::setJoint(QString j)
{
    QMutexLocker locker(&mutex);
    this->joint=j;
    qDebug()<<"Setting joint name to :"<<j;
}

void QNode::stopRobot()
{
  //cmdvel.linear.x = cmdvel.angular.z = 0.0;
  //pub_.publish(cmdvel);
}
}  // namespace learningjoint
