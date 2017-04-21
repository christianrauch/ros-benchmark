#include <ros/ros.h>
#include <benchmark/ByteVectorStamped.h>

void pongCallback(const benchmark::ByteVectorStampedConstPtr &msg) {
    const ros::Duration delay = ros::Time::now() - msg->time;
    ROS_INFO_STREAM("RTT: " << delay.nsec << " ns");
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "ping");

    ros::NodeHandle n;

    ros::Publisher ping_pub = n.advertise<benchmark::ByteVectorStamped>("ping", 1);

    ros::Subscriber pong_sub = n.subscribe("pong", 1, pongCallback);

    ros::Rate loop_rate(1);

    while (ros::ok()) {
        benchmark::ByteVectorStamped msg;

        msg.time = ros::Time::now();

        ping_pub.publish(msg);
        
        ros::spinOnce();

        loop_rate.sleep();
    }
}
