/**
 * @file /include/learningjoint/main_window.hpp
 *
 * @brief Qt based joint for learningjoint.
 *
 * @date November 2010
 **/
#ifndef learningjoint_MAIN_WINDOW_H
#define learningjoint_MAIN_WINDOW_H

/*****************************************************************************
** Includes
*****************************************************************************/

#include <QtGui/QMainWindow>
#include "ui_main_window.h"
#include "qnode.hpp"
#include <string.h>


/*****************************************************************************
** Namespace
*****************************************************************************/

namespace learningjoint {

/*****************************************************************************
** Interface [MainWindow]
*****************************************************************************/
/**
 * @brief Qt central, all operations relating to the view part here.
 */
class MainWindow : public QMainWindow {
Q_OBJECT

public:
    MainWindow(int argc, char** argv, QWidget *parent = 0);
    ~MainWindow();

    void ReadSettings(); // Load up qt program settings at startup
    void WriteSettings(); // Save qt program settings when closing

    QNode qnode;

    void closeEvent(QCloseEvent *event); // Overloaded function
    void showNoMasterMessage();
   void keyPressEvent(QKeyEvent *e);
public Q_SLOTS:
   void radioButton_setjoint();
   void radioButton_2_setjoint();
   void radioButton_3_setjoint();
   void radioButton_4_setjoint();
   void radioButton_5_setjoint();
   void setGripper();
   void setGrasp();

    /******************************************
    ** Auto-connections (connectSlotsByName())
    *******************************************/
    void on_actionAbout_triggered();
    void on_button_connect_clicked(bool check );


   // void on_button_connect_clicked(bool check );
    void on_checkbox_use_environment_stateChanged(int state);
    void on_quit_button_clicked();
   // void on_button_clicked(bool check );


   /* void on_joint_one_clicked();
    void on_joint_two_clicked();
    void on_joint_three_clicked();
    void on_joint_four_clicked();
    void on_joint_five_clicked();*/


    //void on_radiobutton_radiobutton_stateChanged(int state);


    /******************************************
    ** Manual connections
    *******************************************/
    //void updateLoggingView(); // no idea why this can't connect automatically

private:
    Ui::MainWindowDesign ui;
};

}  // namespace learningjoint

#endif // learningjoint_MAIN_WINDOW_H
