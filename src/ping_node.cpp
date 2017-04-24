#include <ros/ros.h>
#include <benchmark/ByteVectorStamped.h>
#include <std_msgs/Float64.h>

class DelayPublischer {
public:
    DelayPublischer(const ros::Publisher &pub) : pub(pub) { }

    void pongCallback(const benchmark::ByteVectorStampedConstPtr &msg) {
        const ros::Duration delay = ros::Time::now() - msg->time;
        ROS_INFO_STREAM("RTT: " << delay.nsec << " ns");
        std_msgs::Float64 rtt;
        rtt.data = delay.toSec();
        pub.publish(rtt);
    }

private:
    const ros::Publisher &pub;
};

int main(int argc, char **argv) {
    ros::init(argc, argv, "ping");

    ros::NodeHandle n;

    ros::Publisher ping_pub = n.advertise<benchmark::ByteVectorStamped>("ping", 1);

    ros::Publisher delay_pub = n.advertise<std_msgs::Float64>("roundtrip_time", 1);

    DelayPublischer dpub(delay_pub);

    ros::Subscriber pong_sub = n.subscribe("pong", 1, &DelayPublischer::pongCallback, &dpub);

    ros::Rate loop_rate(1);

    while (ros::ok()) {
        benchmark::ByteVectorStamped msg;

        msg.time = ros::Time::now();

        ping_pub.publish(msg);

        ros::spinOnce();

        loop_rate.sleep();
    }
}
