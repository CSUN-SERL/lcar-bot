
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
    
    void AddVehicle(int id);
    void DeleteVehicle(int id);
    
    int NumVehicles();
    int NumUGVs();
    int NumQuadRotors();
    int NumOctoRotors();
    int NumVTOLs();
    
    const QMap<int, QString>& GetInitRequests();
    
    QString VehicleStringFromId(int id);
    QString VehicleStringFromMachineName(QString& name);
    int GenerateId(const QString& machine_name);
    int VehicleTypeFromId(int id);

signals:
    //todo add slot to connect to this signal in GCS
    void NotifyOperator(QString message);
    void RemoveInitRequest(int vehicle_id);
    
public slots:
    void OnOperatorInitRequested(const int vehicle_id);
    // todo add all main gui button slots
    
private:
    
    bool OnVehicleInitRequested(lcar_msgs::InitRequest::Request& req,lcar_msgs::InitRequest::Response& res);
    bool OnInitFinalAck(lcar_msgs::InitFinalAck::Request& req, lcar_msgs::InitFinalAck::Response& res);
    
    
    QMap<int/*VehicleType*/, QMap<int, VehicleControl*>> db; //the database
    QMap<int, QString> init_requests; //vehicle initialization requests, storing machine_name and potential id
    
    ros::NodeHandle nh;
    ros::ServiceServer srv_init_request;
    ros::Publisher pub_init_response;
    ros::ServiceServer srv_init_final_ack;
    
    int UGV_ID,
        QUAD_ID,
        OCTO_ID,
        VTOL_ID;
};

}

#endif /* VEHICLEMANAGER_H */

