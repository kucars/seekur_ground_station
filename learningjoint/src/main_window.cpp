/**
 * @file /src/main_window.cpp
 *
 * @brief Implementation for the qt joint.
 *
 * @date February 2011
 **/
/*****************************************************************************
** Includes
*****************************************************************************/

#include <QtGui>
#include <QMessageBox>
#include <iostream>
#include "../include/learningjoint/main_window.hpp"
#include <QPushButton>
#include <QRadioButton>
#include <QDebug>
#include "../include/learningjoint/qnode.hpp"
#include <string.h>

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace learningjoint {

using namespace Qt;
using namespace std;
/*****************************************************************************
** Implementation [MainWindow]
*****************************************************************************/
void MainWindow::keyPressEvent(QKeyEvent *e)
{
    QString string1="";
    //cout<<"ammmmmm hereeeeee";

    double *m = qnode.jointpostion();
   // double m = qnode.jointpostion();
   // cout<<m[1];

    if(e->key() == Qt::Key_Up)
    {
        //remove zero
        qnode.keyPressed(UP);

        qDebug()<<"keyPressed "<<e->key();
    }

    else if(e->key() == Qt::Key_Down)
    {
        qnode.keyPressed(DOWN);
        qDebug()<<"keyPressed "<<e->key();
    }




    QString string2 = "joint 1 \t joint 2 \t joint 3 \t joint 4 \t joint 5 \t Gripper\n";

   for (int i=0;i<5;i++)
    {
        string1 = string1+QString::number(m[i])+"\t";//qt visualizers seen view //radio checks//regidtry q settings
    }

   // string1=QString::number(m)+"*#@*";

   string1=string1+"closed";

    ui.postion_view->setText(string2 + string1);

//    double num = qnode.speed();
//
//     QString n="The linear velocity is: ";
//     QString o = "\n";
//     QString p = "The anguler velocity is: ";
//     double num2 = qnode.speed2();
//     QString f= QString::number(num2);
//     ui.label_5->setText(n+m+o+p+f);
//    //jointspostion();

}

MainWindow::MainWindow(int argc, char** argv, QWidget *parent)
    : QMainWindow(parent), qnode(argc,argv)
{
    ui.setupUi(this); // Calling this incidentally connects all ui's triggers to on_...() callbacks in this class creates user interface.
    QObject::connect(ui.actionAbout_Qt, SIGNAL(triggered(bool)), qApp, SLOT(aboutQt())); // qApp is a global variable for the application


    QObject::connect(ui.radiojoint,SIGNAL(clicked()), this, SLOT(radioButton_setjoint()));
    QObject::connect(this->ui.radiojoint_2,SIGNAL(clicked()), this, SLOT(radioButton_2_setjoint()));
    QObject::connect(this->ui.radiojoint_3,SIGNAL(clicked()), this, SLOT(radioButton_3_setjoint()));
    QObject::connect(this->ui.radiojoint_4,SIGNAL(clicked()), this, SLOT(radioButton_4_setjoint()));
    QObject::connect(this->ui.radiojoint_5,SIGNAL(clicked()), this, SLOT(radioButton_5_setjoint()));
    QObject::connect(this->ui.gripper,SIGNAL(clicked()), this, SLOT(setGripper()));
    QObject::connect(this->ui.grasp_position,SIGNAL(clicked()), this, SLOT(setGrasp()));



    //ui.push->setText("HELOOOO");

    //QPushButton *button = new QPushButton ();
   // button->setText("HELLLO");
this->setFocusPolicy(Qt::StrongFocus);
    ReadSettings();
    setWindowIcon(QIcon(":/images/icon.png"));
    ui.tab_manager->setCurrentIndex(0); // ensure the first tab is showing - qt-designer should have this already hardwired, but often loses it (settings?).
    QObject::connect(&qnode, SIGNAL(rosShutdown()), this, SLOT(close()));






    /*********************
    ** Logging
    **********************/
    //ui.view_logging->setModel(qnode.loggingModel());
    //QObject::connect(&qnode, SIGNAL(loggingUpdated()), this, SLOT(updateLoggingView()));

    /*********************
    ** Auto Start
    **********************/
    if ( ui.checkbox_remember_settings->isChecked() ) {
        //on_button_connect_clicked(true);
    }
}

MainWindow::~MainWindow() {}

/*****************************************************************************
** Implementation [Slots]
*****************************************************************************/

void MainWindow::showNoMasterMessage() {
    QMessageBox msgBox;
    msgBox.setText("Couldn't find the ros master.");
    msgBox.exec();
    close();
}

/*
 * These triggers whenever the button is clicked, regardless of whether it
 * is already checked or not.
 */

void MainWindow::on_button_connect_clicked(bool check ) {

    qDebug()<<"IN CONNECT BUTTON";
    qnode.init();
}




/*void MainWindow::on_joint_one_clicked() {

    qnode.setJoint("j1");
}
void MainWindow::on_joint_two_clicked() {

   qnode.setJoint("j2");
}
void MainWindow::on_joint_three_clicked() {

   qnode.setJoint("j3");
}
void MainWindow::on_joint_four_clicked() {

   qnode.setJoint("j4");
}
void MainWindow::on_joint_five_clicked() {

   qnode.setJoint("j5");
}
*/

void MainWindow::radioButton_setjoint() {
    if(ui.radiojoint->isChecked())
    {
        qnode.setJoint("j1");
    }
}
void MainWindow::radioButton_2_setjoint() {

    if(ui.radiojoint_2->isChecked())
    {
        qDebug()<<"IN radioButton_2_setjoint()";
        qnode.setJoint("j2");
    }
}
void MainWindow::radioButton_3_setjoint() {

    if(ui.radiojoint_3->isChecked())
    {
        qnode.setJoint("j3");
    }
}
void MainWindow::radioButton_4_setjoint() {
    if(ui.radiojoint_4->isChecked())
    {
        qnode.setJoint("j4");
    }
}
void MainWindow::radioButton_5_setjoint() {

    if(ui.radiojoint_5->isChecked())
    {
        qnode.setJoint("j5");
    }}

void MainWindow::setGripper() {

    if(ui.gripper->isChecked())
    {
        qnode.setJoint("g");
    }}

void MainWindow::setGrasp() {

    if(ui.grasp_position->isChecked())
    {
        qnode.setJoint("grasp");
    }}

void MainWindow::on_quit_button_clicked()
{
    qnode.stopRobot();
}

void MainWindow::on_checkbox_use_environment_stateChanged(int state) {
    bool enabled;
    if ( state == 0 ) {
        enabled = true;
    } else {
        enabled = false;
    }
    ui.line_edit_master->setEnabled(enabled);
    ui.line_edit_host->setEnabled(enabled);
}

/*****************************************************************************
** Implemenation [Slots][manually connected]
*****************************************************************************/

/**
 * This function is signalled by the underlying model. When the model changes,
 * this will drop the cursor down to the last line in the QListview to ensure
 * the user can always see the latest log message.
 */
/*void MainWindow::updateLoggingView() {
        ui.view_logging->scrollToBottom();
}*/

/*****************************************************************************
** Implementation [Menu]
*****************************************************************************/

void MainWindow::on_actionAbout_triggered() {
    QMessageBox::about(this, tr("About ..."),tr("<h2>PACKAGE_NAME Test Program 0.10</h2><p>Copyright Yujin Robot</p><p>This package needs an about description.</p>"));
}

/*****************************************************************************
** Implementation [Configuration]
*****************************************************************************/

void MainWindow::ReadSettings() {
    QSettings settings("Qt-Ros Package", "learningjoint");
    restoreGeometry(settings.value("geometry").toByteArray());
    restoreState(settings.value("windowState").toByteArray());
    QString master_url = settings.value("master_url",QString("http://192.168.1.2:11311/")).toString();
    QString host_url = settings.value("host_url", QString("192.168.1.3")).toString();
    //QString topic_name = settings.value("topic_name", QString("/chatter")).toString();
    ui.line_edit_master->setText(master_url);
    ui.line_edit_host->setText(host_url);
    //ui.line_edit_topic->setText(topic_name);
    bool remember = settings.value("remember_settings", false).toBool();
    ui.checkbox_remember_settings->setChecked(remember);
    bool checked = settings.value("use_environment_variables", false).toBool();
    ui.checkbox_use_environment->setChecked(checked);
    if ( checked ) {
        ui.line_edit_master->setEnabled(false);
        ui.line_edit_host->setEnabled(false);
        //ui.line_edit_topic->setEnabled(false);
    }
}

void MainWindow::WriteSettings() {
    QSettings settings("Qt-Ros Package", "learningjoint");
    settings.setValue("master_url",ui.line_edit_master->text());
    settings.setValue("host_url",ui.line_edit_host->text());
    //settings.setValue("topic_name",ui.line_edit_topic->text());
    settings.setValue("use_environment_variables",QVariant(ui.checkbox_use_environment->isChecked()));
    settings.setValue("geometry", saveGeometry());
    settings.setValue("windowState", saveState());
    settings.setValue("remember_settings",QVariant(ui.checkbox_remember_settings->isChecked()));

}

void MainWindow::closeEvent(QCloseEvent *event)
{
    WriteSettings();
    QMainWindow::closeEvent(event);
}

}  // namespace learningjoint
