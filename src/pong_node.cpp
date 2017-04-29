#include <ros/ros.h>
#include <benchmark/ByteVectorStamped.h>

class PingPong {
public:
    PingPong(const ros::Publisher &pub) : pub(pub) { }

    void pingCallback(const benchmark::ByteVectorStampedConstPtr &msg) {
        pub.publish(msg);
    }

private:
    const ros::Publisher &pub;
};

int main(int argc, char **argv) {
    ros::init(argc, argv, "pong");

    ros::NodeHandle n;

    const ros::Publisher pong_pub = n.advertise<benchmark::ByteVectorStamped>("pong", 1);

    PingPong pingping(pong_pub);

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

    const ros::Subscriber ping_sub = n.subscribe("ping", 1, &PingPong::pingCallback, &pingping, transport);

    ros::spin();
}
