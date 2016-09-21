/*
 * File:   main.cpp
 * Author: n8
 *
 * Created on September 20, 2016, 12:07 PM
 */

#include <QApplication>
#include "rqt_gcs/simple_gcs.h"

int main(int argc, char *argv[])
{
    // initialize resources, if needed
    // Q_INIT_RESOURCE(resfile);

    QApplication app(argc, argv);
    
    ros::init(argc, argv, "ISLURP_GCS");
    ros::AsyncSpinner spinner(2);
    spinner.start();
    
    rqt_gcs::SimpleGCS gcs;
    gcs.show();
    
    return app.exec();
}
