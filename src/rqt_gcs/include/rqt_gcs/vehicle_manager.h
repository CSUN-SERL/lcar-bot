
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
#include "lcar_msgs/InitFinalAck.h"

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
    QString VehicleTypeFromName(QString& name);
    const QMap<int, QString>& GetInitRequests();
    int IdFromMachineName(const QString& machine_name);
    
    int NumVehicles();
    int NumUGVs();
    int NumQuadRotors();
    int NumOctoRotors();
    int NumVTOLs();

signals:
    //todo add slot to connect to this signal
    void NotifyOperator();
    void RemoveInitRequest(int vehicle_id);
    
public slots:
    void OnOperatorInitRequested(const int& vehicle_id);
    // todo add all main gui button slots
    
private:
    VehicleControl* EraseVehicleFromDB(int id);
    bool OnVehicleInitRequested(lcar_msgs::InitRequest::Request& req,lcar_msgs::InitRequest::Response& res);
    bool OnInitFinalAck(lcar_msgs::InitFinalAck::Request& req, lcar_msgs::InitFinalAck::Response& res);
    
    QMap<int, VehicleControl*> db; //the database
    QMap<int, QString> init_requests; //vehicle initialization requests, storing machine_name and potential id
    
    ros::NodeHandle nh;
    ros::ServiceServer init_request;
    ros::Publisher init_response;
    ros::ServiceServer init_final_ack;
    
    int NUM_UGV, 
        NUM_QUAD,
        NUM_OCTO,
        NUM_VTOL;
};

}

#endif /* VEHICLEMANAGER_H */

