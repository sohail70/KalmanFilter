#include<ros/ros.h>
#include<sensor_msgs/LaserScan.h>
#include<vector>
#include<nav_msgs/Odometry.h>
#include<random>
#include<chrono>


void gen_normal(std::vector<double>& vec, double mu , double sigma)
{
    //Problem: If you don't initiazlie generator with time(0) it will give you the same random number-->https://stackoverflow.com/questions/69095609/c-gaussian-random-number-generator-keeps-generating-same-sequence
    // Use this site too: https://www.techiedelight.com/generate-random-numbers-cpp/
    unsigned seed = std::chrono::system_clock::now().time_since_epoch().count();
    
    std::mt19937 generator{seed};

    std::normal_distribution<double> distribution(mu,sigma*sigma);
    if (vec.empty()) //The vector must be initialized before getting thorugh the for loop below
        vec.resize(1,0);

    if (isinf(mu))
        return;

    for (auto& v : vec)
    {
        double number = distribution(generator);
        v = number;
        //ROS_INFO("Generated Gaussian random number: %f",v);
    }
}


class LaserLocalization
{
    public:
        LaserLocalization()
        {
            this->wall_position = 4;
            lidar_sub = nh.subscribe("base_scan" , 10 ,&LaserLocalization::laser_callback ,this);
            ray_localization_pub = nh.advertise<nav_msgs::Odometry>("laser_ray_localization",1000);
        }

        void laser_callback(const sensor_msgs::LaserScan::ConstPtr& data)
        {
            sensor_msgs::LaserScan r;
            front_ray = data->ranges.at(0);
        }

        void publish_laser_localization()
        {
            double x = wall_position - front_ray;
            std::vector<double> gaus;
            double alpha = 0.05;
            double std_dev = alpha * front_ray;
            gen_normal(gaus,x,std_dev);
            //ROS_ERROR("REAL LOCALIZATION VS NOISY LOCAL: %f %f",x,gaus[0]);
            localization_x.pose.pose.position.x = gaus[0];
            localization_x.header.stamp = ros::Time::now();
            ray_localization_pub.publish(localization_x);
        }

    private:
        ros::NodeHandle nh;
        ros::Subscriber lidar_sub;
        ros::Publisher ray_localization_pub;
        double front_ray;
        double wall_position;

        nav_msgs::Odometry localization_x;


};



int main(int argc , char** argv)
{
    ros::init(argc , argv , "ray_localization_node");

    LaserLocalization laserLocalization;
    ros::Rate rate(10);
    while (ros::ok)
    {
        laserLocalization.publish_laser_localization();
        ros::spinOnce();
        rate.sleep();
    }
    

}