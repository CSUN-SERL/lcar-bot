/*
 * File:   main.cpp
 * Author: n8
 *
 * Created on September 20, 2016, 12:07 PM
 */

#include <QApplication>
#include "rqt_gcs/gcs.h"


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
    
    ros::init(argc, argv, "ISLURP_GCS");
    ros::AsyncSpinner spinner(2);
    spinner.start();
    
    rqt_gcs::GCS gcs;
    gcs.showMaximized();
    
    return app.exec();
}
