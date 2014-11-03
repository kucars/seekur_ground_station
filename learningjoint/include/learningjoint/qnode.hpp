
/**
 * @file /include/learningjoint/qnode.hpp
 *
 * @brief Communications central!
 *
 * @date February 2011
 **/
/*****************************************************************************
** Ifdefs
*****************************************************************************/

#ifndef learningjoint_QNODE_HPP_
#define learningjoint_QNODE_HPP_

/*****************************************************************************
** Includes
*****************************************************************************/

#include <ros/ros.h>
#include <QThread>
#include <QStringListModel>
#include <QMutexLocker>
#include <QDebug>
#include <string.h>
#include <string>
#include <QString>

#include <termios.h>
#include <signal.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <sys/poll.h>

#include <boost/thread/thread.hpp>
#include <sensor_msgs/JointState.h>

//#include "joy/Joy.h"
#include <geometry_msgs/Twist.h>
/*****************************************************************************
** Namespaces
*****************************************************************************/
enum{
    UP=0,
    DOWN,//auto incerement
    RIGHTB,
    LEFTB,
    SPACEB,
    SHIFTB,
    CTRLB};
namespace learningjoint {

/*****************************************************************************
** Class
*****************************************************************************/

class QNode : public QThread {
    Q_OBJECT
public:
    QNode();

    QNode(int argc, char** argv );
    virtual ~QNode();
    bool init();
     //  bool init(const std::string &master_url, const std::string &host_url);
        void run();
        void keyPressed(int keyId);
        //double x;
        //int speed();
    //what we added
        sensor_msgs::JointState j;

        QString joint;
 double i[5];
    //    ros::NodeHandle n_;
          ros::Publisher pub_;
       // ros::Subscriber joy_sub_;
        int keyId;
        //void joyCallback(const sensor_msgs::Joy::ConstPtr& joy);
       // void joyCallback(const geometry_msgs::Twist::ConstPtr& joy);
double* jointpostion();

        void stopRobot();
      //  void keyboardLoop();
        QMutex mutex;
        bool isKeyPressed;
        double max_tv_;
        double max_rv_;
        /*********************
        ** Logging
        **********************/
        enum LogLevel {
                 Debug,
                 Info,
                 Warn,
                 Error,
                 Fatal
         };
        QStringListModel* loggingModel() { return &logging_model; }
           void log( const LogLevel &level, const std::string &msg);
           void setJoint(QString j);
       Q_SIGNALS:
           void loggingUpdated();
           void rosShutdown();

       private:
           int init_argc;
           char** init_argv;
           ros::Publisher chatter_publisher;
               QStringListModel logging_model;
       };

       }  // namespace learningjoint

       #endif /* learningjoint_QNODE_HPP_ */
