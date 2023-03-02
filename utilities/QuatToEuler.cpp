#include <ros/ros.h>            
#include <tf/tf.h>                  // Transformations for math
#include <sensor_msgs/Imu.h>        // Imu topic type

class Converter {
    private:
    ros::Publisher eulerAngles;     // Publisher variable
    ros::Subscriber gazeboPose;     // Subscriber variable
    geometry_msgs::Vector3 rpy;     // Variable to be published

    public:
    Converter(ros::NodeHandle *n){
        gazeboPose = n->subscribe("/teeterbot/imu",10,&Converter::imu_callback,this);
        eulerAngles = n->advertise<geometry_msgs::Vector3>("/teeterbot/rpy",10);
    }

    void imu_callback(const sensor_msgs::Imu::ConstPtr data){
        //ROS_INFO_STREAM("Input-x=" << data->orientation.x << " | y= " << data->orientation.y << " | z= " << data->orientation.z << " | w= " << data->orientation.w);
        
        tf::Quaternion q(data->orientation.x,
                         data->orientation.y,
                         data->orientation.z,
                         data->orientation.w);
        tf::Matrix3x3 m(q);
        m.getRPY(rpy.x,rpy.y,rpy.z);
        //ROS_INFO_STREAM("roll= "<<rpy.x<<" pitch= "<< rpy.y << " yaw=" << rpy.z);
        eulerAngles.publish(rpy);
    }
};


int main(int argc, char** argv){
    ros::init(argc,argv, "QuatToEuler");
    ros::NodeHandle n;
    Converter c = Converter(&n);
    ros::spin();
}