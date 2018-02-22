#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>

#include <iostream>
#include <vector>
#include <math.h>

class WallDetector{
    private:
        ros::Subscriber scan_sub;

    public:
        WallDetector(ros::NodeHandle &nh){
            scan_sub = nh.subscribe("/scan",1,&WallDetector::callback, this);
        }
        void callback( const sensor_msgs::LaserScanConstPtr& scanMsg){
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
              if(!isnan(ranges[i]))
                r5 = r5 * count/(count+1) + ranges[i]/(count+1);
            }
            j+=n;
            count=0;
            for ( int i=j; i<j+n; i++ )
            {
              if(!isnan(ranges[i]))
                r4 = r4 * count/(count+1) + ranges[i]/(count+1);
            }
            j+=n;
            count=0;
            for ( int i=j; i<j+n; i++ )
            {
              if(!isnan(ranges[i]))
                r3 = r3 * count/(count+1) + ranges[i]/(count+1);
            }
            j+=n;
            count=0;
            for ( int i=j; i<j+n; i++ )
            {
              if(!isnan(ranges[i]))
                r2 = r2 * count/(count+1) + ranges[i]/(count+1);
            }
            j+=n;
            count=0;
            for ( int i=j; i<j+n; i++ )
            {
              if(!isnan(ranges[i]))
                fr = fr * count/(count+1) + ranges[i]/(count+1);
            }
            j+=n;
            count=0;
            for ( int i=j; i<j+n; i++ )
            {
              if(!isnan(ranges[i]))
                fl = fl * count/(count+1) + ranges[i]/(count+1);
            }
            j+=n;
            count=0;
            for ( int i=j; i<j+n; i++ )
            {
              if(!isnan(ranges[i]))
                l2 = l2 * count/(count+1) + ranges[i]/(count+1);
            }
            j+=n;
            count=0;
            for ( int i=j; i<j+n; i++ )
            {
              if(!isnan(ranges[i]))
                l3 = l3 * count/(count+1) + ranges[i]/(count+1);
            }
            j+=n;
            count=0;
            for ( int i=j; i<j+n; i++ )
            {
              if(!isnan(ranges[i]))
                l4 = l4 * count/(count+1) + ranges[i]/(count+1);
            }
            j+=n;
            count=0;
            for ( int i=j; i<j+n; i++ )
            {
              if(!isnan(ranges[i]))
                l5 = l5 * count/(count+1) + ranges[i]/(count+1);
            }
            j+=n;
            count=0;
            for ( int i=j; i<ranges.size(); i++ )
            {
              if(!isnan(ranges[i]))
                l6 = l6 * count/(count+1) + ranges[i]/(count+1);
            }

            if( fl < 0.7 && fr < 0.7 )
            {
              front_0 = true;
            }
            else
            {
              if ( l5 < 1.5 && l4 < 1.5 && l5 < l4 )
                left_1 = true;
              if ( fl < 1.6 && fr < 1.6)
                front_1 = true;
              if ( r5 < 1.5 && r4 < 1.5 && r5 < r4 )
                right_1 = true;
            }


            std::cout<< std::setprecision(2) << std::fixed;
            std::cout << j
                      << " e:" << r5
                      << " d:" << r4
                      << " c:" << r3
                      << " b:" << r2
                      << " a:" << fr
                      << " a:" << fl
                      << " b:" << l2
                      << " c:" << l3
                      << " d:" << l4
                      << " e:" << l5
                      << " f:" << l6 << std::endl;
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
