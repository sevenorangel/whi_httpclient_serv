#include "ros/ros.h"
#include <iostream>
//#include "whi_httpclient_serv/httpclient.h"
#include <cstdlib>


std::vector<std::string> stringSplit(const std::string& str, char delim) {
    std::vector<std::string> elems;
    auto lastPos = str.find_first_not_of(delim, 0);
    auto pos = str.find_first_of(delim, lastPos);
    while (pos != std::string::npos || lastPos != std::string::npos) {
        elems.push_back(str.substr(lastPos, pos - lastPos));
        lastPos = str.find_first_not_of(delim, pos);
        pos = str.find_first_of(delim, lastPos);
    }
    return elems;
}


int main(int argc, char **argv)
{
    setlocale(LC_ALL,"");
  ros::init(argc, argv, "http_client_client");
  if (argc != 5)
  {
    ROS_INFO("usage: http_client host port postorget param1 ");
    return 1;
  }

  ros::NodeHandle n;
//  ros::ServiceClient client = n.serviceClient<whi_httpclient_serv::httpclient>("http_client");
  



  return 0;
}