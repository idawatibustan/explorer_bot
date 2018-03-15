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
            double range_min = scanMsg->range_min;
            double range_max = scanMsg->range_max;

            int j=0, temp_c=0, n=42;

            int r_len = ranges.size();
            int r_wd = r_len/n + 1;
            std::cout << "len:" << r_len
                      << " wd:" << r_wd << std::endl;
            double scan[r_wd] = {0};
            int count[r_wd] = {0};

            for (int i=0; i< r_len; i++) {
              // std::cout << "[" << std::setw(2) << i/n
              //           << "] " << i << std::endl;
              if(!std::isnan(ranges[i]) ) {
                j = i/n;
                temp_c = count[j];
                scan[j] = scan[j] * temp_c/(temp_c+1) + ranges[i]/(temp_c+1);
                count[j] += 1;
              }
            }
            int cen = r_wd/2;
            for (int i=r_wd-1; i >= 0; i--) {
              std::cout << std::setw(2) << i
                        << "," << count[i]
                        << "," << std::setw(4) << scan[i] << std::endl;
              // if(i == cen || i == cen-1 ){
              //   std::cout << " * " << std::endl;
              // }
              // else {
              //   std::cout << std::endl;
              // }
            }
            // perception logic of the walls V2
            double right_outer = scan[2];
            double right_inner = scan[3];

            double front_r = scan[6];
            double front_f = scan[7];
            double front_l = scan[8];

            double left_inner = scan[13];
            double left_outer = scan[14];

            double lim_inner = 1.5;
            double lim_outer = 1.3;
            double lim_front = 0.6;
            double lim_2front = 1.6;

            // direct front wall
            if ((front_r < lim_front && front_f < lim_front && front_l < lim_front) ||
                (front_r == 0 && front_f == 0 && front_l == 0 &&
                 left_inner < 0.65 && right_inner < 0.65))
            {
              front_0 = true;
              wall_state.data = "1000";
            }
            else
            {
              std::string wall ("0");
              // front left
              if ( left_outer < lim_outer &&
                   left_inner < lim_inner &&
                   left_outer <= left_inner &&
                   left_inner != 0 && left_outer != 0 ) {
                left_1 = true;
                wall += "1";
              } else {
                wall += "0";
              }
              if ( front_r < 1.6 && front_f < 1.6 && front_l < 1.6 && front_r != 0 && front_f != 0 && front_l != 0) {
                front_1 = true;
                wall += "1";
              } else {
                wall += "0";
              }
              // front right
              if ( right_outer < lim_outer &&
                   right_inner < lim_inner &&
                   right_outer < right_inner &&
                   right_inner != 0 && right_outer != 0 ) {
                right_1 = true;
                wall += "1";
              } else {
                wall += "0";
              }
              wall_state.data = wall;
            }

            std::cout << " lo:" << std::setw(4) << left_outer
                      << " li:" << std::setw(4) << left_inner
                      << " fl:" << std::setw(4) << front_l
                      << " ff:" << std::setw(4) << front_f
                      << " fr:" << std::setw(4) << front_r
                      << " ri:" << std::setw(4) << right_inner
                      << " ro:" << std::setw(4) << right_outer
                      << std::endl;
            /*
            for ( int i=j; i<j+n; i++ )
            {
              if(!std::isnan(ranges[i])) {
                r5 = r5 * count/(count+1) + ranges[i]/(count+1);
                count++;
              }
            }
            j+=n;
            count=0;
            for ( int i=j; i<j+n; i++ )
            {
              if(!std::isnan(ranges[i])) {
                r4 = r4 * count/(count+1) + ranges[i]/(count+1);
                count++;
              }
            }
            j+=n;
            count=0;
            for ( int i=j; i<j+n; i++ )
            {
              if(!std::isnan(ranges[i])) {
                r3 = r3 * count/(count+1) + ranges[i]/(count+1);
                count++;
              }
            }
            j+=n;
            count=0;
            for ( int i=j; i<j+n; i++ )
            {
              if(!std::isnan(ranges[i])) {
                r2 = r2 * count/(count+1) + ranges[i]/(count+1);
                count++;
              }
            }
            j+=n;
            count=0;
            for ( int i=j; i<j+n; i++ )
            {
              if(!std::isnan(ranges[i])) {
                fr = fr * count/(count+1) + ranges[i]/(count+1);
                count++;
              }
            }
            j+=n;
            count=0;
            for ( int i=j; i<j+n; i++ )
            {
              if(!std::isnan(ranges[i])) {
                fl = fl * count/(count+1) + ranges[i]/(count+1);
                count++;
              }
            }
            j+=n;
            count=0;
            for ( int i=j; i<j+n; i++ )
            {
              if(!std::isnan(ranges[i])) {
                l2 = l2 * count/(count+1) + ranges[i]/(count+1);
                count++;
              }
            }
            j+=n;
            count=0;
            for ( int i=j; i<j+n; i++ )
            {
              if(!std::isnan(ranges[i])) {
                l3 = l3 * count/(count+1) + ranges[i]/(count+1);
                count++;
              }
            }
            j+=n;
            count=0;
            for ( int i=j; i<j+n; i++ )
            {
              if(!std::isnan(ranges[i])) {
                l4 = l4 * count/(count+1) + ranges[i]/(count+1);
                count++;
              }
            }
            j+=n;
            count=0;
            for ( int i=j; i<j+n; i++ )
            {
              if(!std::isnan(ranges[i])) {
                l5 = l5 * count/(count+1) + ranges[i]/(count+1);
                count++;
              }
            }
            j+=n;
            count=0;
            for ( int i=j; i<ranges.size(); i++ )
            {
              if(!std::isnan(ranges[i])) {
                l6 = l6 * count/(count+1) + ranges[i]/(count+1);
                count++;
              }
            }

            if ((fl < 0.65 && fr < 0.65) || (fl == 0 && fr == 0 && l5 < 0.65 && r5 < 0.65))
            {
              front_0 = true;
              wall_state.data = "1000";
            }
            else
            {
              std::string wall ("0");
              if ( l4 < 1.2 && l3 < 1.4 && l4 < l3 && l4 != 0 && l3 != 0 ) {
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
              if ( r4 < 1.2 && r3 < 1.4 && r4 < r3 && r4 != 0 && r3 != 0 ) {
                right_1 = true;
                wall += "1";
              } else {
                wall += "0";
              }
              wall_state.data = wall;
            }

            */
            wall_pub.publish(wall_state);
            /*
            std::cout << std::setprecision(2) << std::fixed;
            std::cout << " l6:" << l6
                      << " l:" << l5
                      << " l4:" << l4
                      << " l:" << l3
                      << " l2:" << l2
                      << " fl:" << fl
                      << " fr:" << fr
                      << " r2:" << r2
                      << " r:" << r3
                      << " r4:" << r4
                      << " r:" << r5
                      << std::endl;
            */
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
