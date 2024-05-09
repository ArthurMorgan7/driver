#include <ros/ros.h> 
#include <ros/time.h>
#include <serial/serial.h>  //ROS已经内置了的串口包 
#include <std_msgs/String.h>
#include <std_msgs/Empty.h> 
#include <string>
#include <vector>
#include <sstream>
#include <cmath>
#include <cstdlib>//string转化为double
#include <iomanip>//保留有效小数
#include <novatel_gps_msgs/Gpgga.h>


using namespace std;

string strRece = "";
string tmp = "";
string tmp_old = "";

void gpggalong(std::string s , double& lat , double& lon ,float& alt, float& hdop, uint32_t& gps_qual, uint32_t& num_sats);
void llaCallback(const std_msgs::StringConstPtr& msg);

///////////////////////////////// main ///////////////////////////////////
int main(int argc, char** argv)
{
    ros::init(argc, argv, "neg");
    ros::NodeHandle nh;
    ros::Subscriber sub = nh.subscribe("GPS_msg", 1000, llaCallback);
    ros::Publisher LLA_pub = nh.advertise<novatel_gps_msgs::Gpgga>("RTK_LLA",1000);
    
    string gstart = "$G"; 
    string gend = "\r\n";

    ros::Rate loop_rate(2000);    
    while (ros::ok())   
    {
        int start = -1, end = -1;
        ros::spinOnce();
        if(tmp == tmp_old) {
            continue;
        }
        tmp_old = tmp;
        strRece += tmp; 
       
        /* ------------------------------- find start ------------------------------- */
        start = strRece.find(gstart);
        if ( start == -1)
        {
            strRece = "";
            continue;
        }
        /* -------------------------------- find end -------------------------------- */
        else
        {
            end = strRece.find(gend, start);
            if (end == -1){
                strRece = strRece.substr(start);    // 截取与丢弃
                continue;
            }
            else    
            {
                /* --------------------------- All has been finded -------------------------- */
                double lat, lon;
                float alt, hdop;
                uint32_t gps_qual, num_sats;    
                //cout << strRece.substr(start,end+2-start) << endl;
                gpggalong(strRece.substr(start,end+2-start),lat, lon, alt, hdop, gps_qual, num_sats); // +2 是为了把\r\n包括进去   
                novatel_gps_msgs::Gpgga GPS_data;
                GPS_data.header.stamp = ros::Time::now();
                GPS_data.lat = lat;
                GPS_data.lon = lon;
                GPS_data.alt = alt;
                GPS_data.hdop = hdop;
                GPS_data.gps_qual = gps_qual;
                GPS_data.num_sats = num_sats;
                LLA_pub.publish(GPS_data);
                strRece = strRece.substr(end+2);
            }
        }
        loop_rate.sleep();  
    }
}

void gpggalong(std::string s , double& lat , double& lon ,float& alt, float& hdop, uint32_t& gps_qual, uint32_t& num_sats)
{
    //cout << s <<endl;
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


void llaCallback(const std_msgs::StringConstPtr& msg)
{
    tmp = msg->data;
}
 