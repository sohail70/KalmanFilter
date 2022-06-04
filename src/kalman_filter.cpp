/*
1D kalman filter
TODO: measurement data is invalid until you get near the wall so the filter should only rely on odometry
*/

#include<ros/ros.h>
#include<nav_msgs/Odometry.h>
#include<random>
#include<chrono>
#include<vector>
#include<ros/exception.h>
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

void gen_uniform(std::vector<double>& vec , double min , double max)
{
    unsigned seed = std::chrono::system_clock::now().time_since_epoch().count();
    std::mt19937 generator{seed};
    std::uniform_real_distribution<> range_between(min , max);

    if (vec.empty()) //The vector must be initialized before getting thorugh the for loop below
        vec.resize(1,0);

    for (auto& v : vec)
    {
        double number = range_between(generator);
        v= number;
    }
    
    
}

class KalmanFilter{
    public:
        KalmanFilter() {
            this->odom_variance = 4;
            this->meas_variane = 0.05;
            std::vector<double> unif;
            gen_uniform(unif,0,4);
            this->mu = unif[0];
            this->sig = 1000;

            this->last_odom_x = 0;

            noisy_odom_sub = nh.subscribe("odom_noisy",10,&KalmanFilter::noisy_odom_callback,this);
            ray_local_sub = nh.subscribe("laser_ray_localization",10,&KalmanFilter::ray_localization_callback,this);
            filtered_x_pub = nh.advertise<nav_msgs::Odometry>("filtered_odom",10);
            nav_msgs::Odometry::ConstPtr o_msg_1;
            nav_msgs::Odometry::ConstPtr o_msg_2;
            try{
                while(o_msg_1 == nullptr && o_msg_2 ==nullptr)
                {
                    ROS_ERROR("WAITING FOR ODOM MESSAGES");
                    o_msg_1 = ros::topic::waitForMessage<nav_msgs::Odometry>("odom_noisy",ros::Duration(1));
                    o_msg_2 = ros::topic::waitForMessage<nav_msgs::Odometry>("laser_ray_localization",ros::Duration(1));
                }
                ROS_ERROR("ODOM MESSAGES RECEIVED");
            }
            catch (ros::Exception& e)
            {
                ROS_ERROR("NOT YET %s",e.what());
            }

        }


        void noisy_odom_callback(const nav_msgs::Odometry::ConstPtr& data)
        {
            noisy_odom_x = data->pose.pose.position.x ;
        }

        void ray_localization_callback(const nav_msgs::Odometry::ConstPtr& data)
        {
            ray_local_x = data->pose.pose.position.x; //map from measurement to state space
        }


        void prediction_step()
        {
            double motion = noisy_odom_x - last_odom_x;
            mu += motion;
            sig += odom_variance;
            last_odom_x = noisy_odom_x;
            ROS_ERROR("Prediction sig:%f",sig);
        }

        void correction_step()  //double mean1 , double variance1 , double mean2 , double variance2)
        {
            //this->mu = (variance1 * mean2 + variance2 * mean1)/(variance1+variance2);
            //this->sig = 1 / ((1/variance1) + (1/variance2) );

            mu = (sig * ray_local_x + meas_variane * mu) / (sig + meas_variane);
            sig = 1 / ((1/sig) + (1/meas_variane) );
            ROS_ERROR("Filtered x and sigma vs noisy_odom %f %f %f",mu,sig,noisy_odom_x);
        }

        void publish_filtered_x(double mu)
        {
            nav_msgs::Odometry filtered_odom;
            filtered_odom.pose.pose.position.x = mu;
            filtered_odom.header.stamp = ros::Time::now();
            filtered_x_pub.publish(filtered_odom);

        }
        void filter_run()
        {
            prediction_step();
            correction_step();
            publish_filtered_x(mu);
        }



    private:
        ros::NodeHandle nh;
        ros::Subscriber noisy_odom_sub;
        ros::Subscriber ray_local_sub;
        ros::Publisher filtered_x_pub;

        double mu; //filtered position of the robot
        double sig; //filtered uncertainty of the robot

        double odom_variance;  //variance = sigma^2
        double meas_variane; //variance = sigma^2

        double noisy_odom_x;
        double last_odom_x;
        double ray_local_x;

};

int main(int argc , char** argv)
{
    ros::init(argc,argv,"kalman_filter_node");

    KalmanFilter kf;
    ros::Rate rate(5);
    while(ros::ok)
    {
        kf.filter_run();
        ros::spinOnce();
        rate.sleep();
    }
/*
    std::vector<double> vec;
    gen_uniform(vec,0,4);
    ROS_ERROR("EEEEEEEEEEEEEEEEEEEEEEEE %f",vec[0]);
*/
}