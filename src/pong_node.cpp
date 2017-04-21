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

    const ros::Subscriber ping_sub = n.subscribe("ping", 1, &PingPong::pingCallback, &pingping);

    ros::spin();
}
