#include <ros/ros.h> 
#include <ros/time.h>
#include <serial/serial.h> 
#include <std_msgs/String.h>
#include <string>
#include <vector>
#include <sstream>
#include <cmath>
#include <novatel_gps_msgs/Gpgga.h>

using namespace std;

void gpggalong(std::string s , double& lat , double& lon ,float& alt, float& hdop, uint32_t& gps_qual, uint32_t& num_sats)
{
    vector<string> v;
    string::size_type pos1, pos2;
    pos1 = s.find(",") + 1;
    pos2 = s.find(",",pos1);

    while ( string::npos !=pos2 ) 
    {
        v.push_back( s.substr( pos1, pos2-pos1 ) );  
        pos1 = pos2 + 1;       
        pos2 = s.find(",",pos1); 
    }

    if ( pos1 != s.length() ){
        v.push_back( s.substr( pos1 ));
    }
        
    // 维度解析
    if (v[1] != ""){
        lat = std::atof(v[1].c_str());
    }
    else{
        lat = -1.0;
    }
    // 经度解析
    if (v[3] != ""){
        lon = std::atof(v[3].c_str());
    }
    else{
        lon = -1.0;
    }
    // 数据质量解析
    
    if (v[5] != ""){
        gps_qual = atoi(v[5].c_str());
    }
    else{
        gps_qual = 99;
    }
    //cout << gps_qual << endl;


    // 使用中的卫星数量解析
    if (v[6] != ""){
        num_sats = std::atof(v[6].c_str());
    }
    else{
        num_sats = 99;
    }
    
    // 水平经度因子
    if (v[7] != ""){
        hdop = std::atof(v[7].c_str());
    }
    else{
        hdop = -1.0;
    }
    // 高度解析
    if (v[8] != ""){
        alt = std::atof(v[8].c_str());
    }
    else{
        alt = -1.0;
    }

    
    
    
}

int main(int argc, char** argv)
{
    /* ------------------------------- Step 1 初始操作 ------------------------------ */
    ros::init(argc, argv, "test_lla");
    ros::NodeHandle nh;
    ros::Publisher LLA_pub = nh.advertise<novatel_gps_msgs::Gpgga>("RTK_LLA",1000);

    std::string s = "$GPGGA,134658.00,5106.9802863,N,11402.3037304,W,2,09,1.0,1048.234,M,-16.27,M,08,AAAA\r\n";
    std::string gstart = "$G";     // GPS起始标志
    std::string gend = "\r\n";      // GPS终止标志
    double lat, lon;
    float alt, hdop;
    uint32_t gps_qual, num_sats;

    ros::Rate loop_rate(1);    // 控制循环频率为20Hz
    while(ros::ok())
    {
        ros::spinOnce();
        int start = -1, end = -1;
        start = s.find(gstart);
        end = s.find(gend);

        gpggalong(s.substr(start,end+2-start),lat, lon, alt, hdop, gps_qual, num_sats); // +2 是为了把\r\n包括进去   

        novatel_gps_msgs::Gpgga GPS_data;
        GPS_data.header.stamp = ros::Time::now();
        GPS_data.lat = lat;
        GPS_data.lon = lon;
        GPS_data.alt = alt;
        GPS_data.hdop = hdop;
        GPS_data.gps_qual = gps_qual;
        GPS_data.num_sats = num_sats;
        LLA_pub.publish(GPS_data);

        cout << setiosflags(ios::fixed) << setprecision(7)  // 保留小数点后7位
              << "维度：" << lat << ";"
              << "经度：" << lon << ";"
              << "高度：" << alt << ";"
              << "水平精度因子：" << hdop << ";"
              << "定位质量：" << gps_qual << ";"
              << "卫星数量：" << num_sats << ";" << endl;

        loop_rate.sleep();  // 用 sleep 来控制频率，睡眠时间自适应，以达到指定的频率
    }

}
