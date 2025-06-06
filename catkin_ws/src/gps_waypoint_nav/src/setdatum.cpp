#include <ros/ros.h>
#include <ros/package.h>

#include <sensor_msgs/NavSatFix.h>
#include <geographic_msgs/GeoPose.h>
#include <robot_localization/SetDatum.h>

// initialize variables
double latitude, longitude, altitude;
bool initial_gps = true;
int counter = 0;

void gps(const sensor_msgs::NavSatFix::ConstPtr& gps_msg)
{
    // Initial coordinates of Base frame (robot)
    if(initial_gps && counter == 10)
    {
        // if (gps_msg->latitude != 0.0 && gps_msg->longitude != 0.0 && gps_msg->altitude != 0.0 && counter == 5)
        // {
            latitude = gps_msg->latitude;
            longitude = gps_msg->longitude;
            altitude = gps_msg->altitude;
            initial_gps = false;
            //counter++;
        // }
    }
    counter++;

    //Lookup the TF between robot and Map/world


    //ROS_INFO("GPS: Latitude: %.8f, Longitude: %.8f", lati, longi);
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
    srv.request.geo_pose.position.latitude = latitude;
    srv.request.geo_pose.position.longitude = longitude;
    srv.request.geo_pose.position.altitude = altitude;
    srv.request.geo_pose.orientation.x = 0.0;
    srv.request.geo_pose.orientation.y = 0.0;
    srv.request.geo_pose.orientation.z = 0.0;
    srv.request.geo_pose.orientation.w = 1.0;

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