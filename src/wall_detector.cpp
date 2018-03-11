#include <ros/ros.h>
#include <std_msgs/String.h>
#include <sensor_msgs/LaserScan.h>

#include <iostream>
#include <vector>
#include <cmath>

class WallDetector{
    private:
        ros::Subscriber scan_sub;
        ros::Publisher wall_pub;

    public:
        WallDetector(ros::NodeHandle &nh){
            scan_sub = nh.subscribe("/scan",1,&WallDetector::callback, this);
            wall_pub = nh.advertise<std_msgs::String>("/wall_scan",1);
        }
        void callback( const sensor_msgs::LaserScanConstPtr& scanMsg){
            std_msgs::String wall_state;
            bool front_0 = false;
            bool left_1 = false;
            bool front_1 = false;
            bool right_1 = false;
            double r5 = 0, r4 = 0, r3 = 0, r2 = 0;
            double fr = 0, fl = 0;
            double l2 = 0, l3 = 0, l4 = 0, l5 = 0, l6 = 0;

            std::vector<float> ranges = scanMsg->ranges;
            int j=0, count=0, n=61;
            for ( int i=j; i<j+n; i++ )
            {
              if(!std::isnan(ranges[i]))
                r5 = r5 * count/(count+1) + ranges[i]/(count+1);
            }
            j+=n;
            count=0;
            for ( int i=j; i<j+n; i++ )
            {
              if(!std::isnan(ranges[i]))
                r4 = r4 * count/(count+1) + ranges[i]/(count+1);
            }
            j+=n;
            count=0;
            for ( int i=j; i<j+n; i++ )
            {
              if(!std::isnan(ranges[i]))
                r3 = r3 * count/(count+1) + ranges[i]/(count+1);
            }
            j+=n;
            count=0;
            for ( int i=j; i<j+n; i++ )
            {
              if(!std::isnan(ranges[i]))
                r2 = r2 * count/(count+1) + ranges[i]/(count+1);
            }
            j+=n;
            count=0;
            for ( int i=j; i<j+n; i++ )
            {
              if(!std::isnan(ranges[i]))
                fr = fr * count/(count+1) + ranges[i]/(count+1);
            }
            j+=n;
            count=0;
            for ( int i=j; i<j+n; i++ )
            {
              if(!std::isnan(ranges[i]))
                fl = fl * count/(count+1) + ranges[i]/(count+1);
            }
            j+=n;
            count=0;
            for ( int i=j; i<j+n; i++ )
            {
              if(!std::isnan(ranges[i]))
                l2 = l2 * count/(count+1) + ranges[i]/(count+1);
            }
            j+=n;
            count=0;
            for ( int i=j; i<j+n; i++ )
            {
              if(!std::isnan(ranges[i]))
                l3 = l3 * count/(count+1) + ranges[i]/(count+1);
            }
            j+=n;
            count=0;
            for ( int i=j; i<j+n; i++ )
            {
              if(!std::isnan(ranges[i]))
                l4 = l4 * count/(count+1) + ranges[i]/(count+1);
            }
            j+=n;
            count=0;
            for ( int i=j; i<j+n; i++ )
            {
              if(!std::isnan(ranges[i]))
                l5 = l5 * count/(count+1) + ranges[i]/(count+1);
            }
            j+=n;
            count=0;
            for ( int i=j; i<ranges.size(); i++ )
            {
              if(!std::isnan(ranges[i]))
                l6 = l6 * count/(count+1) + ranges[i]/(count+1);
            }

            if ((fl < 0.65 && fr < 0.65) || (fl == 0 && fr == 0 && l5 < 0.65 && r5 < 0.65))
            {
              front_0 = true;
              wall_state.data = "1000";
            }
            else
            {
              std::string wall ("0");
              if ( l5 < 1.5 && l4 < 1.8 && l5 < l4 && l5 != 0 && l4 != 0 ) {
                left_1 = true;
                wall += "1";
              } else {
                wall += "0";
              }
              if ( fl < 1.6 && fr < 1.6 && fl != 0 && fr != 0) {
                front_1 = true;
                wall += "1";
              } else {
                wall += "0";
              }
              if ( r5 < 1.5 && r4 < 1.8 && r5 < r4 && r5 != 0 && r4 != 0 ) {
                right_1 = true;
                wall += "1";
              } else {
                wall += "0";
              }
              wall_state.data = wall;
            }

            wall_pub.publish(wall_state);
            std::cout << std::setprecision(2) << std::fixed;
            std::cout << " r:" << r5
                      << " r4:" << r4
                      << " r:" << r3
                      << " r2:" << r2
                      << " fr:" << fr
                      << " fl:" << fl
                      << " l2:" << l2
                      << " l:" << l3
                      << " l4:" << l4
                      << " l:" << l5
                      << " l6:" << l6 << std::endl;
            std::cout << "front:" << front_0
                      << " left:" << left_1
                      << " frnt:" << front_1
                      << " rght:" << right_1 << std::endl;
        }
        double average( std::vector<float>& v )
        {
            double sum = 0.0;
            int n = v.size();
            for ( int i=0; i<n; i++ )
            {
                sum += v[i];
            }
            return ( sum / n );
        }
};

int main(int argc, char** argv){
    ros::init(argc, argv, "wall_detector");
    ros::NodeHandle nh;
    WallDetector wd(nh);

    ros::spin();
    return 0;
}
