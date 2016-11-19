/* 
 * File:   UI.h
 * Author: darkknight
 *
 * Created on September 9, 2016, 10:01 AM
 */

#ifndef _UI_H
#define	_UI_H

#include <ros/ros.h>
#include <ros/console.h>
#include <std_msgs/String.h>
#include <nav_msgs/Odometry.h>

#include "ui_UI.h"
#include <QTimer>

#include <iostream>
#include <random>
#include <ctime>

class UI : public QMainWindow {
    Q_OBJECT
public:
    UI();
    virtual ~UI();
    QTimer* timer;
public slots:
    void slotTimer();
public:
    ros::NodeHandle *nh;
    ros::Subscriber sub;
	//void odomCallBack(const nav_msgs::OdometryConstPtr &odom);
private:
    Ui::UI widget;
    std::mt19937 engine;
    std::uniform_int_distribution<int> randomColor;
    geometry_msgs::Pose boatPose_enu;
    geometry_msgs::Twist boatTwist_enu; 
    QPixmap imageCircle,imageTriangle,imageCruciform,nullPixmap; 
};

#endif	/* _UI_H */
