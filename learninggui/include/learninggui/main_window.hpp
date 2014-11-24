/**
 * @file /include/learninggui/main_window.hpp
 *
 * @brief Qt based gui for learninggui.
 *
 * @date November 2010
 **/
#ifndef learninggui_MAIN_WINDOW_H
#define learninggui_MAIN_WINDOW_H

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

namespace learninggui {

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
    /******************************************
    ** Auto-connections (connectSlotsByName())
    *******************************************/
    void on_actionAbout_triggered();
   // void on_button_connect_clicked(bool check );
    void on_checkbox_use_environment_stateChanged(int state);
    void on_quit_button_clicked();
    void on_button_clicked(bool check );


    /******************************************
    ** Manual connections
    *******************************************/
    void updateLoggingView(); // no idea why this can't connect automatically
private:
    Ui::MainWindowDesign ui;
};

}  // namespace learninggui

#endif // learninggui_MAIN_WINDOW_H
