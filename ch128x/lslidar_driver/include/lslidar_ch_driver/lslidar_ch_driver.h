/******************************************************************************
 * This file is part of lslidar driver.
 *
 * Copyright 2022 LeiShen Intelligent Authors. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *****************************************************************************/

#ifndef LSLIDAR_Ch_DRIVER_H
#define LSLIDAR_Ch_DRIVER_H
#define DEG_TO_RAD 0.017453292
#define RAD_TO_DEG 57.29577951

#include <unistd.h>
#include <stdio.h>
#include <netinet/in.h>
#include <string>

#include <boost/shared_ptr.hpp>
#include <boost/thread.hpp>
#include <thread>

#include <ros/ros.h>

#include <lslidar_msgs/LslidarChPacket.h>
#include <lslidar_msgs/LslidarChScan.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/LaserScan.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>


#include "input.h"

namespace lslidar_ch_driver {

    static const double DISTANCE_RESOLUTION = 0.0000390625; /**< meters */

    static const double big_angle[32]={-17,-16,-15,-14,-13,-12,-11,-10,
                                       -9,-8,-7,-6,-5,-4.125,-4,-3.125,
                                       -3,-2.125,-2,-1.125,-1,-0.125,0,0.875,
                                       1,1.875,2,3,4,5,6,7
    };
    static const double big_angle_ch128s1[32] = {
            -12, -11, -10, -9, -8, -7, -6, -5,
            -4.125, -4, -3.125, -3, -2.125, -2, -1.125, -1,
            -0.125, 0, 0.875, 1, 1.875, 2, 3, 4,
            5, 6, 7, 8, 9, 10, 11, 12
    };

    //static const double big_angle_ch16x1[16] = {-1.0,-0.75,-0.50,-0.25,0.0,0.25,0.50,0.75,1.0,1.25,1.50,1.75,2.0,2.25,2.50,2.75};
    static const double big_angle_ch16x1[4] = {-1.0,0.0,1.0,2.0};


/*    static const double scan_mirror_altitude[4] = {
            -0.0,
            0.005759586531581287,
            0.011693705988362009,
            0.017453292519943295,
    };

    static const double sin_scan_mirror_altitude[4] = {
            std::sin(scan_mirror_altitude[0]), std::sin(scan_mirror_altitude[1]),
            std::sin(scan_mirror_altitude[2]), std::sin(scan_mirror_altitude[3]),
    };*/
    struct PointXYZITM {
        PCL_ADD_POINT4D
        float intensity;
        uint16_t ring;
        double timestamp;

        EIGEN_MAKE_ALIGNED_OPERATOR_NEW  // make sure our new allocators are aligned
    } EIGEN_ALIGN16;
// enforce SSE padding for correct memory alignment

    class LslidarChDriver {
    private:

/*        union two_bytes{
            int16_t value;
            char bytes[2];
        };

        struct Point {
            uint8_t vertical_line;        //0-127
            uint8_t azimuth_1;
            uint8_t azimuth_2;      ///< 1500-16500, divide by 100 to get degrees
            uint8_t distance_1;
            uint8_t distance_2;
            uint8_t distance_3;
            uint8_t intensity;
        };*/



        struct Firing {
            //double vertical_angle;
            int vertical_line;
            double azimuth;
            double distance;
            float intensity;
            double time;
        };

    public:

        LslidarChDriver(ros::NodeHandle &n, ros::NodeHandle &pn);

        ~LslidarChDriver();

        bool initialize();
        void publishLaserScan();
        void publishPointCloud();

        int convertCoordinate(struct Firing lidardata);

        bool polling();

        void difopPoll(void);

        void initTimeStamp(void);

        bool isPointInRange(const double& distance) {
            return (distance >= min_range && distance <= max_range);
        }

        //void getFPGA_GPSTimeStamp(lslidar_msgs::LslidarChPacketPtr &packet);

        typedef boost::shared_ptr<LslidarChDriver> LslidarChDriverPtr;
        typedef boost::shared_ptr<const LslidarChDriver> LslidarChDriverConstPtr;

    private:


        bool loadParameters();

        bool createRosIO();

        //socket Parameters
        int msop_udp_port;
        int difop_udp_port;

        boost::shared_ptr<Input> msop_input_;
        boost::shared_ptr<Input> difop_input_;

        // Converter convtor_
        boost::shared_ptr<boost::thread> difop_thread_;

        // Ethernet relate variables
        std::string lidar_ip_string;
        std::string group_ip_string;
        std::string frame_id;
        std::string pointcloud_topic;
        std::string lidar_type;

        in_addr lidar_ip;

        int socket_id;

        bool add_multicast;
        bool pcl_type;
        std::string dump_file;
        double prism_angle[4];
        double prism_offset;
        double min_range;
        double max_range;
        double angle_disable_min;
        double angle_disable_max;
        int channel_num;
        int channel_num1;
        int channel_num2;
        int echo_num;

        double horizontal_angle_resolution;


        // ROS related variables
        ros::NodeHandle nh;
        ros::NodeHandle pnh;

        ros::Publisher pointcloud_pub;
        ros::Publisher laserscan_pub;

        lslidar_msgs::LslidarChScanPtr sweep_data;
        lslidar_msgs::LslidarChScanPtr sweep_data_bac;



        // add for time synchronization
        bool use_time_service;
        bool use_gps;
        bool publish_laserscan;
        bool gain_prism_angle;

        uint64_t packet_timestamp_s;
        uint64_t packet_timestamp_ns;
        double packet_timestamp;
        double last_packet_timestamp;
        double point_cloud_timestamp;
        double point_time;
        double packet_rate;



        unsigned char packetTimeStamp[10];
        struct tm cur_time;

     //   ros::Time timeStamp;

     //   ros::Time packet_timeStamp;

        double packet_interval_time;
     //   Firing firings[171];

        double sin_theta_1[128];
        double sin_theta_2[128];
        double cos_theta_1[128];
        double cos_theta_2[128];

        double ch64w_sin_theta_1[128];
        double ch64w_sin_theta_2[128];
        double ch64w_cos_theta_1[128];
        double ch64w_cos_theta_2[128];

        double ch16x1_sin_theta_1[16];
        double ch16x1_sin_theta_2[16];
        double ch16x1_cos_theta_1[16];
        double ch16x1_cos_theta_2[16];
    };

    typedef LslidarChDriver::LslidarChDriverPtr LslidarChDriverPtr;
    typedef LslidarChDriver::LslidarChDriverConstPtr LslidarChDriverConstPtr;
    typedef PointXYZITM VPoint;
    typedef pcl::PointCloud<VPoint> VPointCloud;

} // namespace lslidar_driver

POINT_CLOUD_REGISTER_POINT_STRUCT(lslidar_ch_driver::PointXYZITM,
                                  (float, x, x)(float, y, y)(float, z, z)
                                          (float, intensity, intensity)
                                          (std::uint16_t, ring, ring)
                                          (double, timestamp, timestamp)
)

#endif // _LSLIDAR_Ch_DRIVER_H_
