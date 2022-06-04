/*
Odom noise
Ground truth x from odometry
Adding noise to the x component and publish it to odom_noisy topic.
To be used with 1D kalman filter algorithm


use https://blog.lxsang.me/post/id/16 for std of translation --> The calcs are base on sebastian thrun's book probabilistic robotics



why don't you just change gazebo odom plugin covariance matrix? You can't because its hardcoded https://answers.ros.org/question/353977/gazebo-change-covariance-matrix/
*/

#include<ros/ros.h>
#include<nav_msgs/Odometry.h>
#include<vector>
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

    for (auto& v : vec)
    {
        double number = distribution(generator);
        v = number;
        //ROS_INFO("Generated Gaussian random number: %f",v);

    }
}

class NoisyOdom{
    public: 
        NoisyOdom()
        {
            ROS_WARN("CONSTRUCTORR");

            odom_sub = nh.subscribe<nav_msgs::Odometry>("odom",10,&NoisyOdom::odom_callback,this); //! Don't forget the "this"
            noisy_odom_x_pub = nh.advertise<nav_msgs::Odometry>("odom_noisy",10);
            ground_truth_x_pub = nh.advertise<nav_msgs::Odometry>("ground_truth_x",10);
            last_odom.pose.pose.position.x = 0;
            noisy_odom.pose.pose.position.x = 0;

            while (odom_received ==false)
            {
                ROS_INFO("Waiting for odom message");
                ros::spinOnce();
                ros::Duration(0.05).sleep();
            }
            
            
        }
        void odom_callback(const nav_msgs::Odometry::ConstPtr& data)
        {
            cur_odom = data;
            odom_received = true;
        }

        double standard_dev_calc(double delta_trans)
        {
            double alpha = 4;
            //ROS_WARN("DELTA %f",delta_trans);
            double std_trans = alpha * delta_trans;
            return std_trans;
        }

        void publish_noisy_odom_x()
        {
            nav_msgs::Odometry real_x;
            //ROS_INFO("publihssssssssss: %f",cur_odom->pose.pose.position.x);
            real_x.pose.pose.position.x  = cur_odom->pose.pose.position.x;
            ground_truth_x_pub.publish(real_x);
            
            double delta_trans = cur_odom->pose.pose.position.x - last_odom.pose.pose.position.x;
            //noisy x calc
            std::vector<double> gaus;
            gen_normal(gaus,delta_trans,standard_dev_calc(delta_trans));
            noisy_odom.pose.pose.position.x +=   gaus[0];
            ROS_INFO("Real odom vs noisy odom %f  %f: ",real_x.pose.pose.position.x , noisy_odom.pose.pose.position.x);
            
            //noisy_odom.header.frame_id = "odom";
            //noisy_odom.child_frame_id = "base_footprint";
            noisy_odom.header.stamp = ros::Time::now();
            noisy_odom_x_pub.publish(noisy_odom);
            last_odom.pose = cur_odom->pose;
        }

        bool odom_received = false;
        bool scan_received = false;
    private:
        ros::NodeHandle nh;
        
        ros::Subscriber odom_sub;

        ros::Publisher noisy_odom_x_pub;
        ros::Publisher ground_truth_x_pub;

        nav_msgs::Odometry last_odom;
        nav_msgs::Odometry::ConstPtr cur_odom;
        nav_msgs::Odometry noisy_odom; 

    
};


int main(int argc , char** argv)
{
    ros::init(argc,argv,"noisy_odom_node");

    NoisyOdom noisyOdom;
   
    ros::Rate rate(10);  
    while(ros::ok())
    {
        noisyOdom.publish_noisy_odom_x();
        ros::spinOnce();
        rate.sleep();
    }
    
   
}