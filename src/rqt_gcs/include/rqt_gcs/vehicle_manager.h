
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
#include <std_msgs/Int32.h>
#include <image_transport/image_transport.h>

#include "vehicle/vehicle_control.h"
#include "util/data_types.h"
#include "util/image.h"
#include "lcar_msgs/InitRequest.h"
#include "sensor_msgs/NavSatFix.h"

namespace rqt_gcs
{
    
    //todo implement these forware declarations
    class UGVControl;
    class VTOLControl;
    class NAOCOntrol;
    
    typedef struct ObjectDetectionParameters_
    {
        //defaults
        double hit_thresh = 0; // displayed as a decimal
        int step_size = 16;
        int padding = 8;
        double scale_factor = 1.15; // displayed as a decimal
        bool mean_shift = false;
    } ObjectDetectionParameters;
    
class VehicleManager : public QObject
{
    Q_OBJECT
public:
    VehicleManager(QObject *parent=0);
    virtual~VehicleManager();
    
    void AddVehicle(int v_id);
    void DeleteVehicle(int v_id);
    
    /**
     * @return the number of vehicles in the system
     */
    int NumTotalVehicles();
    /**
     * 
     * @param v_type the VehicleType enum casted to an integer. 
     * @return the number of vehicles in the system of type specified by v_type
     */
    int NumVehiclesByType(int v_type);
    int NumUGVs();
    int NumQuadRotors();
    int NumOctoRotors();
    int NumVTOLs();
    
    /**
     * Accessor function used by VehicleInitWidget to show init. reqests to the operator
     * 
     * @return const reference to the QMap containing the vehicle init. requests
     */
    const QMap<int, QString>& GetInitRequests();
    
    QString VehicleStringFromId(int id);
    int GenerateId(const QString& machine_name);
    int VehicleTypeFromId(int id);

    void SubscribeToImageTopic(QString& topic);
    void AdvertiseObjectDetection();
    
    ObjectDetectionParameters* GetObjectDetectionParams();
    
    //methods for publishing object detection paramerter updates
    void PublishHitThreshold(double thresh);
    void PublishStepSize(int step);
    void PublishPadding(int padding);
    void PublishScaleFactor(double scale);
    void PublishMeanShift(bool on);
    
signals:
    //todo add slot to connect to this signal in GCS
    void NotifyOperator(QString message);
    void AddToInitWidget(QString machine_name, int vehicle_id);
    void AddVehicleWidget(int v_id);
    void DeleteVehicleWidget(int v_id);
    void NewImageFrame(QPixmap& img);
    
public slots:
    void OnOperatorInitResponse(const int vehicle_id);
    // todo add all main gui button slots
    
    //Vehicle Commands//////////////////////////////////////////////////////////
    /**
     * Vehicle commands use ros to transmit messages to vehicles, so they may 
     * be time consuming. these functions should not be called from the GUI thread
     * from the thread that this VehicleManager instance runs on. we achieve this
     * using signals and slots with a queued connection type.
     */
    
    /**
     * \brief sets the way point for vehicle associated with the given internal id
     * @param v_id the vehicles internal id
     * @param location the container for latitude, longitude, and latitude
     */
    void SetWaypoint(int v_id, const sensor_msgs::NavSatFix& location);
    
     /**
     * convenience function: set the way point for the specified vehicle, where
     * vehicle type and number are specified in v_string
     * @param v_string the human readable vehicle type containing its number ("quad1")
     * @param location the container for latitude, longitude, and latitude
     */
    void SetWaypoint(std::string v_string, const sensor_msgs::NavSatFix& location);
    
    /**
     * \brief Arm (or Disarm) the vehicle with given v_id
     *  
     * @param v_id the vehicle id of the vehicle you want to arm
     * @param value whether or not to arm the vehicle
     */
    void Arm(int v_id, bool value);
    
    /**
     * \brief voice command overloaded function for @Arm(int v_id, bool value)
     * @param v_string the vehicle string to arm, eg "quad1, ground vehicle1"
     * @param value whether or not to arm the vehicle
     */
    void Arm(std::string v_string, bool value);
    
    /**
     * \brief Set the UAV Flight Mode for UAV(quad, octo, or vtol) with given id.
     *        Note: this function assumes UAV type vehicles such
     *        VehicleType::[quad_rotor|octo_rotor|vtol] and will fail if not one 
     *        of these.
     * 
     * @param the vehicle id of the UAV to send the command to
     * @param mode Mode to Set: Choose from Stabilize, Alt Hold, Auto, Guided,
     *        Loiter, RTL, or Circle
    */
    void SetFlightMode(int v_id, std::string mode);
    
    /**
     * \brief voice command overloaded function for @SetFlightMode(int v_id, std::string mode)
     * @param v_string the vehicle string to set flight mode for, eg "quad1, husky1"
     * @param mode Mode to Set: Choose from Stabilize, Alt Hold, Auto, Guided,
     *        Loiter, RTL, or Circle
     */
    void SetFlightMode(std::string v_string, std::string mode);
    
    //todo the rest of the vehicle commands
    //end VehicleCommands///////////////////////////////////////////////////////
    
    
    //Vehicle Info queries//////////////////////////////////////////////////////

    /**
     * \brief retrieve information like state, battery, and mission progress about
     *        for the specified UGV (eg, a vehicle of type VehicleType::ugv)
     * @param ugv_id the id of the UGV to get info about
     * @return UGVInfoPtr the info container. will be NULL if vehicle isn't found
     */
    UGVInfoPtr GetUGVInfo(int ugv_id);
    
     /**
     * \brief retrieve information like state, battery, and mission progress about
     *        for the specified UAV (eg, a vehicle of type VehicleType::[quad-rotor | octo-rotor | vtol]
     * @param ugv_id the id of the UAV to get info about
     * @return UAVInfoPtr the info container. will be NULL if vehicle isn't found
     */
    UAVInfoPtr GetUAVInfo(int uav_id);
    
    //end Vehicle info queries//////////////////////////////////////////////////
    
    
    //ros related///////////////////////////////////////////////////////////////
    void ImageCallback(const sensor_msgs::ImageConstPtr& msg);
    void ReceivedObjectDetectionRequest(const std_msgs::Int32ConstPtr& msg);
    
private:
    
    void InitSettings();
    
    /*
     * \brief the ros ServiceServer callback that for service InitRequests
     */
    bool OnVehicleInitRequested(lcar_msgs::InitRequest::Request& req,lcar_msgs::InitRequest::Response& res);
    
    /*
    * \brief convenience function for voice recognition software. when a voice command is 
    * issued for a given vehicle, say "quad1" parse that into its internal id
    * before issuing the command.
    * 
    * @param the string containing vehicle type and order in the system (eg "quad1" or "octo2")
    */
    int IdfromVehicleString(QString v_type);
    
    /*
     * this is the database containing all the vehicles.
     * its a double nested QMap where the outer maps' key is the VehicleType casted to an integer,
     * whose assoicated value is itself a QMap. There is one map per VALID vehicle type. the inner maps
     * have the vehicle id as the key, and a VehicleControl* as the value. 
     */
    QMap<int/*VehicleType*/, QMap<int, VehicleControl*>> db; //the database
    
    /*
     * contains all the vehicle init. requests. see @OnVehicleInitRequested().
     */
    QMap<int, QString> init_requests; //vehicle init. requests, storing machine_name and potential id
    
    
    ros::NodeHandle nh;
    ros::ServiceServer srv_init_request;
    ros::Publisher pub_init_response;
    
    image_transport::ImageTransport it_stereo;
    image_transport::Subscriber sub_stereo;
    
    int UGV_ID,
        QUAD_ID,
        OCTO_ID,
        VTOL_ID;
    
    
    struct ObjectDetectionMessageHandlers // publishers and subscribers
    {
        ros::Publisher pub_hit_thresh;
        ros::Publisher pub_step_size;
        ros::Publisher pub_padding;
        ros::Publisher pub_scale_factor;
        ros::Publisher pub_mean_shift;
        ros::Subscriber sub_od_request;
    } od_handlers;

    ObjectDetectionParameters od_params;
};

}

#endif /* VEHICLEMANAGER_H */

