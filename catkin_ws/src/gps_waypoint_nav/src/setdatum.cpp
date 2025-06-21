#include <ros/ros.h>
#include <ros/package.h>

#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/Imu.h>
#include <geographic_msgs/GeoPose.h>
#include <robot_localization/SetDatum.h>
#include <math.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>


// initialize variables
double latitude, longitude, altitude;
bool initial_gps = true;
int counter = 0;
double imu_ori_x = 0, imu_ori_y = 0, imu_ori_z = 0, imu_ori_w = 0;
double qx = 0, qy = 0, qz =0, qw = 0;

void gps(const sensor_msgs::NavSatFix::ConstPtr& gps_msg)
{
    // Initial coordinates of Base frame (robot)
    // if(initial_gps && counter >= 20)
    // {
    //     if (gps_msg->latitude != 0.0 && gps_msg->longitude != 0.0 && gps_msg->altitude != 0.0)
    //     {
            latitude = 40.186697;
            longitude = -8.417771;
            altitude = 0;
            initial_gps = false;
            // 40.186944, -8.417771

            // 40.186697, -8.417771
            //counter++;
    //     }
    //     else 
    //     {
    //         ROS_INFO("GPS not valid, waiting...");
    //         initial_gps = true;
    //     }
    // }
    // counter++;

    //Lookup the TF between robot and Map/world


    //ROS_INFO("GPS: Latitude: %.8f, Longitude: %.8f", lati, longi);
}

void imu(const sensor_msgs::Imu::ConstPtr& imu_msg)
{
    double roll = 0;
    double pitch = 0;
    double yaw = 3.14;
	const double PI  = 3.141592653589793238463;

    double cy = cos(yaw * 0.5);
    double sy = sin(yaw * 0.5);
    double cp = cos(pitch * 0.5);
    double sp = sin(pitch * 0.5);
    double cr = cos(roll * 0.5);
    double sr = sin(roll * 0.5);

    qw = cr * cp * cy + sr * sp * sy;
    qx = sr * cp * cy - cr * sp * sy;
    qy = cr * sp * cy + sr * cp * sy;
    qz = cr * cp * sy - sr * sp * cy;

    imu_ori_x = imu_msg->orientation.x;
    imu_ori_y = imu_msg->orientation.y;
    imu_ori_z = imu_msg->orientation.z;
    imu_ori_w = imu_msg->orientation.w;
    // imu_ori_x = qx;
    // imu_ori_y = qy;
    // imu_ori_z = qz;
    // imu_ori_w = qw;

}

bool setDatum(ros::NodeHandle& nh, double latitude, double longitude, double altitude)
{
    // Create a service client for the /set_datum service
    ros::ServiceClient client = nh.serviceClient<robot_localization::SetDatum>("/gps_waypoint_nav/datum");

    // Wait for the service to become available
    if (!client.waitForExistence(ros::Duration(5.0)))
    {
        ROS_ERROR("Service /gps_waypoint_nav/datum not available");
        return false;
    }

    // Create a request and response object
    robot_localization::SetDatum srv;
    srv.request.geo_pose.position.latitude = 40.186697;
    srv.request.geo_pose.position.longitude = -8.417771;
    srv.request.geo_pose.position.altitude = 0;
    // srv.request.geo_pose.orientation.x = -0.009;
    // srv.request.geo_pose.orientation.y = -0.019;
    // srv.request.geo_pose.orientation.z = 0.547;
    // srv.request.geo_pose.orientation.w = 0.837;

    srv.request.geo_pose.orientation.x = 0;
    srv.request.geo_pose.orientation.y = 0;
    srv.request.geo_pose.orientation.z = 0;
    srv.request.geo_pose.orientation.w = 1;

    // srv.request.geo_pose.orientation.x = imu_ori_x;
    // srv.request.geo_pose.orientation.y = imu_ori_y;
    // srv.request.geo_pose.orientation.z = imu_ori_z;
    // srv.request.geo_pose.orientation.w = imu_ori_w;

    // Call the service
    if (client.call(srv))
    {
        ROS_INFO("Datum set successfully");
        return true;
    }
    else
    {
        ROS_ERROR("Failed to call service /gps_waypoint_nav/datum");
        return false;
    }
}


int main(int argc, char** argv)
{
    ros::init(argc, argv, "setdatum"); //initiate node called gps_waypoint
    ros::NodeHandle n;
    ROS_INFO("Initiated setdatum node 1");

    ros::Subscriber sub_gps = n.subscribe("/reach/fix", 1000, gps);
    ros::Subscriber sub_imu = n.subscribe("/madgwick_filtered_imu", 1000, imu);

    while(ros::ok())
    {
        if(!initial_gps)
        {
            // Set the datum
            if (!setDatum(n, latitude, longitude, altitude))
            {
                ROS_ERROR("Failed to set datum");
            }else{
                ROS_INFO("Datum set successfully");
                return 0;
            }
        }
        ros::spinOnce();
    }
}