/*
 * File:   main.cpp
 * Author: n8
 *
 * Created on September 20, 2016, 12:07 PM
 */

#include <QApplication>
#include <QThread>

#include "util/strings.h"
#include "util/debug.h"
#include "gcs/gcs_main_window.h"
#include "gcs/vehicle_manager.h"

bool LockThisPC()
{
    //todo create islurp.pid to enforce this application as single instance
    return true;
}

bool ROSLockThisNetwork()
{
    //todo query master node to determine if ISLURP_GCS is already running
    //to enforce only one instance of this application for a given network
    return true;
}

int main(int argc, char *argv[])
{
    // initialize resources, if needed
    // Q_INIT_RESOURCE(resfile);

    QApplication app(argc, argv);
    
    
    if(!LockThisPC())
        return -1;
    
    if(!ROSLockThisNetwork())
        return -2;
    
    
    ros::init(argc, argv, "GCS");
    ros::AsyncSpinner spinner(0); // use all processor cores and threads
    spinner.start();
    
    gcs::dbg::InitDbg();
    gcs::InitStrings(); // initializes application global strings
    
    gcs::VehicleManager vm;
    QThread background;
    vm.moveToThread(&background);
    vm.ConnectToUIAdapter();
    background.start();
    
    gcs::GCSMainWindow gcs(&vm);
    gcs.showMaximized();
    
    int ret = app.exec();
    
    background.quit();
    ros::shutdown();
    return ret;
}
