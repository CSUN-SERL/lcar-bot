
/* 
 * File:   vehicle_manager.h
 * Author: n8
 *
 * Created on September 8, 2016, 11:41 AM
 */

#ifndef VEHICLEMANAGER_H
#define VEHICLEMANAGER_H

#include <QMap>
#include <QSet>
#include <QObject>

#include <ros/ros.h>

#include "vehicle/vehicle_control.h"
#include "util/data_types.h"
#include "lcar_msgs/InitRequest.h"

namespace rqt_gcs
{
    
class VehicleManager : public QObject
{
    Q_OBJECT
public:
    VehicleManager(QObject *parent=0);
    virtual~VehicleManager();
    
    void AddVehicleById(int id);
    void AddUGV(int id);
    void AddQuadRotor(int id);
    void AddOctoRotor(int id);
    void AddVTOL(int id);
    
    void DeleteUGV(int id);
    void DeleteQuadRotor(int id);
    void DeleteOctoRotor(int id);
    void DeleteVTOL(int id);
    
    QString TypeStringFromId(int id);
    const QList<QString> GetInitRequests();
    int IdFromMachineName(const QString& machine_name);
    
    int NumVehicles();
    int NumUGVs();
    int NumQuadRotors();
    int NumOctoRotors();
    int NumVTOLs();

signals:
    //todo add slot to connect to this signal
    void NotifyOperator();
    
public slots:
    void OnOperatorInitRequested(const QString& machine_name);
    // todo add all main gui button slots
    
private:
    VehicleControl* EraseVehicleFromDB(int id);
    bool OnVehicleInitRequested(lcar_msgs::InitRequest::Request& req, const lcar_msgs::InitRequest::Response& res);
    
    QMap<int, VehicleControl*> db; //the database
    QSet<QString> init_requests; //vehicle initialization requests, storing machine_name
    
    ros::NodeHandle nh;
    ros::ServiceServer init_request_server;
    ros::ServiceClient init_response_client;
    
    int NUM_UGV, 
        NUM_QUAD,
        NUM_OCTO,
        NUM_VTOL;
};

}

#endif /* VEHICLEMANAGER_H */

