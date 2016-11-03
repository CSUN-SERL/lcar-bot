
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
#include <QProcess>

#include <ros/ros.h>

#include "vehicle/vehicle_control.h"
#include "util/data_types.h"
#include "lcar_msgs/InitRequest.h"
#include "sensor_msgs/NavSatFix.h"

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
    
    //Vehicle Commands//////////////////////////////////////////////////////////
    
    /*
     * convenience function: set the way point for the specified vehicle, where
     * vehicle type and number are specified in v_string
     * @param v_string the human readable vehicle type containing its number ("quad1")
     * @param location the container for latitude, longitude, and latitude
     */
    void SetWaypoint(std::string v_string, const sensor_msgs::NavSatFix& location);
    
    /*
     * sets the way point for vehicle associated with the given internal id
     * @param v_id the vehicles internal id
     * @param location the container for latitude, longitude, and latitude
     */
    void SetWaypoint(int v_id, const sensor_msgs::NavSatFix& location);
    
    
    
    
    int NumVehicles();
    int NumUGVs();
    int NumQuadRotors();
    int NumOctoRotors();
    int NumVTOLs();
    
    const QMap<int, QString>& GetInitRequests();
    
    QString VehicleStringFromId(int id);
    int GenerateId(const QString& machine_name);
    int VehicleTypeFromId(int id);

signals:
    //todo add slot to connect to this signal in GCS
    void NotifyOperator(QString message);
    void AddToInitWidget(QString machine_name, int vehicle_id);
    
public slots:
    void OnOperatorInitResponse(const int vehicle_id);
    // todo add all main gui button slots
    
private:
    
    bool OnVehicleInitRequested(lcar_msgs::InitRequest::Request& req,lcar_msgs::InitRequest::Response& res);
    int IdfromVehicleString(QString v_type);
    
    QMap<int/*VehicleType*/, QMap<int, VehicleControl*>> db; //the database
    QMap<int, QString> init_requests; //vehicle initialization requests, storing machine_name and potential id
    
    ros::NodeHandle nh;
    ros::ServiceServer srv_init_request;
    ros::Publisher pub_init_response;
    
    int UGV_ID,
        QUAD_ID,
        OCTO_ID,
        VTOL_ID;
};

}

#endif /* VEHICLEMANAGER_H */

