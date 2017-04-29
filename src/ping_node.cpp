#include <ros/ros.h>
#include <benchmark/ByteVectorStamped.h>
#include <std_msgs/Float64.h>
#include <thread>

class DelayPublischer {
public:
    DelayPublischer(const ros::Publisher &pub) : pub(pub) { }

    void pongCallback(const benchmark::ByteVectorStampedConstPtr &msg) {
        const ros::Duration delay = ros::Time::now() - msg->time;
        ROS_INFO_STREAM("RTT: " << delay.toSec() << " s");
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

    // configure
    std::string transport_protocol_name;
    if(!n.param<std::string>("transport", transport_protocol_name, "udp"))
        ROS_INFO_STREAM("No 'transport' given, using udp by default.");
    else
        ROS_INFO_STREAM("Transport set to: " << transport_protocol_name);

    // set connetcion type
    ros::TransportHints transport;
    if(transport_protocol_name.compare("udp"))
        transport = ros::TransportHints().udp();
    else if(transport_protocol_name.compare("tcp"))
        transport = ros::TransportHints().tcp();

    ros::Publisher ping_pub = n.advertise<benchmark::ByteVectorStamped>("ping", 1);

    ros::Publisher delay_pub = n.advertise<std_msgs::Float64>("roundtrip_time", 1);

    DelayPublischer dpub(delay_pub);

    ros::Subscriber pong_sub = n.subscribe("pong", 1, &DelayPublischer::pongCallback, &dpub, transport);

    std::thread spin_thread( [](){ ros::spin(); } );

    ros::Rate loop_rate(1);

    while (ros::ok()) {
        benchmark::ByteVectorStamped msg;

        msg.time = ros::Time::now();

        ping_pub.publish(msg);

        loop_rate.sleep();
    }

    spin_thread.join();
}
