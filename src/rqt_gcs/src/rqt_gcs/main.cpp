/*
 * File:   main.cpp
 * Author: n8
 *
 * Created on September 20, 2016, 12:07 PM
 */

#include <QApplication>
#include <QThread>

#include "util/strings.h"
#include "rqt_gcs/gcs.h"
#include "rqt_gcs/vehicle_manager.h"

namespace rqt_gcs
{                        // these are defined globally in util/strings.h
    QString COMPANY; 
    QString APPLICATION;
}

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
    
    rqt_gcs::COMPANY = "SERL";
    rqt_gcs::APPLICATION = "LCAR_Bot";
    
    ros::init(argc, argv, "GCS");
    ros::AsyncSpinner spinner(0); // use all processor cores
    spinner.start();
    
    rqt_gcs::VehicleManager vm;
    QThread background;
    vm.moveToThread(&background);
    vm.ConnectToUIAdapter();
    background.start();
    
    rqt_gcs::GCS gcs(&vm);
    gcs.showMaximized();
    
    return app.exec();
}
