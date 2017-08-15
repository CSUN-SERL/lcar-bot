#ifndef UAV_CONTROL
#define UAV_CONTROL

#include <stdio.h>
#include <vector>
#include <tf/tf.h>
#include <stdlib.h>
#include <string.h>
#include <ros/ros.h>
#include <termios.h>
#include <angles/angles.h>
#include <tf/transform_datatypes.h>
#include <eigen_conversions/eigen_msg.h>

#include <sensor_msgs/Imu.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/BatteryState.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Float64.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/Vector3Stamped.h>
#include "lcar_msgs/AccessPointStamped.h"
#include "lcar_msgs/TargetLocal.h"
#include "lcar_msgs/TargetGlobal.h"
#include "vehicle/mavros_helper.h"

namespace gcs
{

#define QUEUE_SIZE 100            // Message Queue size for publishers
#define TRAVEL_WT 0.5
#define SCOUT_WT 0.5
// #define THRESHOLD_XY 0.08
// #define THRESHOLD_Z 0.08
#define THRESHOLD_XY 0.1
#define THRESHOLD_Z 0.15
#define THRESHOLD_XY_GPS 0.00001
#define THRESHOLD_Z_GPS 0.5
#define THRESHOLD_YAW 5 //degrees
#define THRESHOLD_GPS 0.001        //Lat & Lon tolerances
#define THRESHOLD_ALT 1            //Altitude tolerance for GPS
#define THRESHOLD_DEPTH 2
#define THRESHOLD_LAND_TIME 2
#define THRESHOLD_WAYPOINT_TIME 1
#define ALT_RTL 2
#define BATTERY_MIN 0.10    //Minimum battery level for RTL
#define DEF_NS "V"
#define R_EARTH 6371        //Earth's radius in km

#define RAD2DEG (180/PI)
#define DEG2RAD (PI/180)
    
class UAVControl : public MavrosHelper
{
public:

    UAVControl(int uav_id);
    ~UAVControl();

    /**
     * \brief Set online mode on or off
     *
     * @param value: pass true for online, false for offline
     */
    bool SetOnlineMode(bool value)
    {
       online_mode = value;
    }
    /**
      Executes a Scout Building play using local coordinates

      @param target_point lcar_msgs::TargetLocal building location

    */
    void ScoutBuilding(lcar_msgs::TargetLocal msg_target);
    
    /**
     Sets the pose_target
     * @param target is new target location
     */
    void SetTarget(geometry_msgs::Pose& target);
    
    void SetTarget(double lat, double lng, double alt); 
    
    void SetMission(geometry_msgs::Pose& target, double radius);
    virtual void SetMission(std::vector<geometry_msgs::Pose> waypoints_list) override;
    /**
     * Moves vehicle to target x,y location
     * @param x
     * @param y
     */
    void TravelToTargetPosition();
    
    /**
     * Moves vehicle to target z altitude
     * @param z
     */
    void TravelToTargetAltitude();
    
    /**
     * Turns the vehicle to desired angle. target_angle in degrees.
     * @param target_angle
     */
    void TurnToAngle();
    
    nav_msgs::Path FollowWaypoints(std::vector<geometry_msgs::Pose> target_waypoints);
    void SetTargetAltitude(double z);
    void SetTargetPosition(double x,double y);
    void SetTargetAngle(geometry_msgs::Quaternion angle);
    
    /*relative functions should be called once to prevent infinite movement*/
    void TravelRelativeToPosition(double x,double y);
    void TurnRelative(double degrees);
    void TravelRelativeToAltitude(double z);
    void StrafeX(double x);
    void StrafeY(double y);
    
    /**
      Executes a Scout Building play using global coordinates

      @param target_point lcar_msgs::TargetGlobal building location

    */
    void ScoutBuilding(lcar_msgs::TargetGlobal msg_target);

    /**
      Manage the UAV and ensure that it is stable
    */
    void Run();
    /**
      Manage the UAV and ensure that it is stable
    */
    void SendDoorResponse(lcar_msgs::Query msg_answer) { pub_door_answer.publish(msg_answer); }

    /*!
     * \brief Manages heartbeat emission
     * \param drop_connection True or false
     *
     * Simulates a drop in connection to the ground control station and stops
     * emitting heartbeats.
     */
    void DropConnection(bool drop_connection)
    {
        connection_dropped = drop_connection;
    }
    
    void StartMission() override;

    /*!
     * \brief Pause the current mission
     *
     * Stops the UAV at its current location and waits for a command from the
     * GCS to resume the mission.
     */
    void PauseMission() override;

    /*!
     * \brief Resumes the previously paused mission.
     *
     * Starts the mission that was previously paused. If there was no previously
     * paused mission, does nothing.
     */
    void ResumeMission() override;

    /*!
     * \brief Cancels the current mission
     *
     * Cancels the current mission and commands the UAV to return base.
     */
    void StopMission() override;
    
    /*!
     * \brief Cancels the current mission
     * \param flight_mode The flight mode to switch to after cancelling mission
     *
     * Cancels the current mission and changes flight mode to the one selected
     * by the user.
     */
    void StopMission(std::string flight_mode);
    
    void SetRev(int rev)                                                {scout_rev = rev;}
    
    //Getter Functions
    FlightState GetFlightState()                                        { return UpdateFlightState(); }
    int GetDistanceToWP() override                                      { return CalculateDistance(pose_target, pose_local); }
    float GetMissionProgress() override;
    std::vector<lcar_msgs::AccessPointStampedPtr>* GetRefAccessPoints() { return &access_pts; }
    std::vector<lcar_msgs::QueryPtr>* GetDoorQueries()                  { return &queries_door; }
    Mode getMode()                                                      { return goal; }

    void Land();
    void TakeOff(double alt);
    
    void fakeQuery(const sensor_msgs::Image& image, int building_id, int wall);
    void fakeQuery(const sensor_msgs::ImageConstPtr& msg, int building_id, int wall);
    
    virtual Position getPosition() override;
    
    virtual bool MissionComplete() override
    {
        return cur_waypoint >= path_mission.poses.size();
    }
    
    bool canQuery()
    {
        return can_query;
    }
    
private:
    void InitialSetup();

    //Private Class Functions
    /**
      Compare two geometry_msgs::Point objects within a threshold value.

      @param point1  First point to compare
      @param point2  Second point to compare
    */
    int ComparePosition(geometry_msgs::Pose pose1, geometry_msgs::Pose pose2);

    /**
      Compare two geometry_msgs::Point objects within a threshold value. Only looks at altitude.

      @param point1  First point to compare
      @param point2  Second point to compare
    */
    int CompareAltitude(geometry_msgs::Pose pose1, geometry_msgs::Pose pose2);

    /**
      Calculate the distance between two points.

      @param point1  First point
      @param point2  Second point
    */
    int CalculateDistance(geometry_msgs::Pose pose1, geometry_msgs::Pose pose2);

    /**
      Calculate a Vector3d object that defines the displacement for reaching a
      point on a circle.

      @param target_point   Center point for target building path generation
    */
    nav_msgs::Path CircleShape(lcar_msgs::TargetLocal target_point);

    /**
      Calculate a list of waypoints in a circular shape around the target GPS location

      @param target_point   Center point of the circle
      @return Waypoint Mission item.
    */
    mavros_msgs::WaypointPush CircleShape(lcar_msgs::TargetGlobal target_point);

    /**
      Check critical sensor values
    */
    void SafetyCheck();
    
   /**

    Compares two angles of pose inputs. returns 0 if they are within tolerance
    */
    int CompareYaw(geometry_msgs::Pose pose1, geometry_msgs::Pose pose2);
    
    /**
    Compares two degree angles. returns 0 if they are within tolerance
    */
    int CompareYaw(double yaw1, double yaw2);
    
     /**
     * Moves the drone to specific location x,y,z and yaw
      * set before by call to SetTarget or TravelTo____
     */
    void TravelToTargetLocation();
    
    /**
      Manage a local mission
    */
    void RunLocal();

    /**
      Manage a global mission
    */
    void RunGlobal();

    //Callback Prototypes
    void DepthCallback(const std_msgs::Float64& msg_depth){ object_distance = msg_depth;}
    //void DoorQueryCallback(const lcar_msgs::Query& msg_query){ queries_door.push_back(msg_query); }
    void DetectionCallback(const lcar_msgs::QueryPtr& msg)
    {
        lcar_msgs::AccessPointStampedPtr new_point =
                            boost::make_shared<lcar_msgs::AccessPointStamped>();

        new_point->header = msg->img_framed.header;
        new_point->ap.query = *msg;
        new_point->ap.altitude = altitude_rel.data;
        new_point->ap.compass_heading = heading_deg.data;
        new_point->ap.location = pos_global;
        new_point->ap.ap_type = lcar_msgs::AccessPoint::DOOR;
        access_pts.push_back(new_point);


        if(online_mode && msg->img_framed.header.seq % 5 == 0)
            queries_door.push_back(msg);

   //quad1.MoveToLocation();
    }

    void UavHeartbeatCallback(std_msgs::Int32 heartbeat_msg)
    {
        if(connection_dropped)
            return;

        timer_heartbeat_uav.stop();

        heartbeat_recieved = true;
        uav_heartbeat.data = heartbeat_msg.data;

        timer_heartbeat_uav.start();
    }

    void UavHeartbeatTimeoutCallback(const ros::TimerEvent& e)
    {
        heartbeat_recieved = false;
    }

    void GcsPublishHeartbeat(const ros::TimerEvent& e)
    {
        gcs_heartbeat.data++;
        pub_heartbeat.publish(gcs_heartbeat);
    }
    
    double GetYaw(const geometry_msgs::Pose& pose);
    RPY getOrientationDegrees(const geometry_msgs::Pose& pose);
    
            
    //For returning Flight State Data to GCS
    FlightState UpdateFlightState();

    //ROS NodeHandle, Service Client, Publisher, and Subscriber Variables
    ros::Publisher                  pub_door_answer;
    ros::Subscriber                 sub_vrpn,
                                    sub_depth,
                                    sub_door_query,
                                    sub_detection,
                                    sub_heartbeat;

    //UAV State Variables
    geometry_msgs::Pose             pose_target,
                                    pose_home,
                                    pose_previous;
    nav_msgs::Path                  path_mission; 
    std_msgs::Float64               object_distance;
    
    ros::Time                       last_request;
    ros::Time                       land_check;
    ros::Time                       waypoint_check;
    std::vector<lcar_msgs::AccessPointStampedPtr>        access_pts;
    std::vector<lcar_msgs::QueryPtr> queries_door;
    bool                            collision = false,
                                    online_mode = true;
    int                             tries = 0,
                                    scout_rev = 1;
    bool                            can_query = false;
};

}
#endif
