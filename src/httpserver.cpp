#include "ros/ros.h"
#include "httplib.h"


int main(int argc, char **argv)
{
    ros::init(argc, argv, "http_server");
    ros::NodeHandle n;

    //ros::ServiceServer service = n.advertiseService("add_two_ints", add);
    ROS_INFO("Ready to httpserver.");
    //ros::spin();

    // HTTP
    httplib::Server svr;

    // HTTPS
    //httplib::SSLServer svr;

    svr.Get("/hi", [](const httplib::Request &, httplib::Response &res) {
    res.set_content("Hello World!", "text/plain");
    });

    svr.listen("0.0.0.0", 8080);



    return 0;
}