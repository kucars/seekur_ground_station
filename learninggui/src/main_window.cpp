/**
 * @file /src/main_window.cpp
 *
 * @brief Implementation for the qt gui.
 *
 * @date February 2011
 **/
/*****************************************************************************
** Includes
*****************************************************************************/

#include <QtGui>
#include <QMessageBox>
#include <iostream>
#include "../include/learninggui/main_window.hpp"
#include <QPushButton>
#include <QDebug>
#include "../include/learninggui/qnode.hpp"
#include <string.h>
#include <QString>
#include <string>

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace learninggui {

using namespace Qt;
using namespace std;
/*****************************************************************************
** Implementation [MainWindow]
*****************************************************************************/
void MainWindow::keyPressEvent(QKeyEvent *e)
{
   // qDebug()<<"keyPressed "<<e->key();
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

    else if(e->key() == Qt::Key_Right)
    {
        qnode.keyPressed(RIGHTB);
        qDebug()<<"keyPressed "<<e->key();
    }
    else if(e->key() == Qt::Key_Left)
    {
        //qDebug("am in left");
        qnode.keyPressed(LEFTB);
        qDebug()<<"keyPressed "<<e->key();
    }
    else if (e->key() == Qt::Key_Space)
    {
        qnode.keyPressed(SPACEB);
        qDebug()<<"keyPressed "<<e->key();
    }
    else if (e->key() == Qt::Key_Shift)
    {
        //qDebug("am in shift");
        qnode.keyPressed(SHIFTB);
        qDebug()<<"keyPressed "<<e->key();
    }
    else if (e->key() == Qt::Key_Control)
    {
       // qDebug("am in CTRL");
        qnode.keyPressed(CTRLB);
        qDebug()<<"keyPressed "<<e->key();
//    }
//    else if (e->modifiers().testFlag(Qt::ControlModifier))
//    {
//             if(e->key() == Qt::Key_Down)
//    {
//        qDebug("am in CTRL+UP");
//        qnode.keyPressed(CTRLB);
//        //qDebug()<<"keyPressed 1"<<e->key();
//        //qDebug()<<"keyPressed 2"<<e->modifiers();

//    }
}
//to print the speed
    double num = qnode.speed();
     QString m= QString::number(num);
     QString n="The linear velocity is: ";
     QString o = "\n";
     QString p = "The anguler velocity is: ";
     double num2 = qnode.speed2();
     QString f= QString::number(num2);
    // ui.label_5->setText(n+m+o+p+f);
    //speed();


}

MainWindow::MainWindow(int argc, char** argv, QWidget *parent)
    : QMainWindow(parent), qnode(argc,argv)
{
    ui.setupUi(this); // Calling this incidentally connects all ui's triggers to on_...() callbacks in this class creates user interface.
    QObject::connect(ui.actionAbout_Qt, SIGNAL(triggered(bool)), qApp, SLOT(aboutQt())); // qApp is a global variable for the application

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
    ui.view_logging->setModel(qnode.loggingModel());
    QObject::connect(&qnode, SIGNAL(loggingUpdated()), this, SLOT(updateLoggingView()));
    //ui.label_5
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

void MainWindow::on_button_clicked(bool check ) {

    qnode.init();
}


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
void MainWindow::updateLoggingView() {
        ui.view_logging->scrollToBottom();
}

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
    QSettings settings("Qt-Ros Package", "learninggui");
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
    QSettings settings("Qt-Ros Package", "learninggui");
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

}  // namespace learninggui
