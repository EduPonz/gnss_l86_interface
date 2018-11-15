#include "gnss_l86_interface/GnssData.h"
#include "ros/ros.h"
#include "gps_lib.h"

int main(int argc, char **argv) 
{
    GPSInterface gps;

    if (!gps.open_connection("/dev/serial0", 9600))
        return 1;

    ros::init(argc, argv, "gnss_l86_interface_node");
    ros::NodeHandle n;
    ros::Publisher publisher = n.advertise<gnss_l86_interface::GnssData>("gnss_data", 1000);
    ros::Rate loop_rate(1);

    int num_lines = 0;
    gnss_l86_interface::GnssData gnss_data;

    while (ros::ok())
    {
        num_lines = gps.read_lines();
        position last_position = gps.get_position();

        if (num_lines > 0)
        {
            gnss_data.latitude = last_position.latitude;
            gnss_data.longitude = last_position.longitude;
            gnss_data.fix = last_position.fix;
            gnss_data.number_of_satelites = last_position.number_of_satelites;
            gnss_data.horizontal_precision = last_position.horizontal_precision;
            gnss_data.altitude = last_position.altitude;
            gnss_data.timestamp = last_position.timestamp;
        }
        else
        {
            gnss_data.latitude = 0;
            gnss_data.longitude = 0;
            gnss_data.fix = 0;
            gnss_data.number_of_satelites = 0;
            gnss_data.horizontal_precision = 0;
            gnss_data.altitude = 0;
            gnss_data.timestamp = last_position.timestamp;
        }

        publisher.publish(gnss_data);
        ros::spinOnce();
        loop_rate.sleep();
    }
    gps.close_connection();

    return 0;
}
