/******************************************************************
node to handle httpclient get request

Features:
- http get request
-

Written by Yue Zhou, sevendull@163.com

Apache License Version 2.0, check LICENSE for more information.
All text above must be included in any redistribution.

Changelog:
2024-03-19: Initial version
2024-xx-xx: xxx
******************************************************************/
#include "ros/ros.h"
#include "httplib.h"
#include <iostream>
#include "std_srvs/SetBool.h"
#include <nlohmann/json.hpp>

std::string host;
int port;

//开始动作的请求  ，  "开始放片" 和 "开始抓片" 的请求
bool StartActionRequst(httplib::Client & cli,std::string apistr)
{
    ros::Rate rate(1) ;
    httplib::Params params;
    std::string getStr= apistr;
    httplib::Headers headers = {
    { "Accept", "application/json" }
    };
    if(apistr=="place")
    {
        getStr="/api/placing";
    }else if (apistr=="reclaim")
    {
        getStr="/api/reclaiming";
    }

    while(true)
    {
        if (auto getres = cli.Get(getStr,params,headers,[](uint64_t len, uint64_t total) {
            return true; 
            }))
        {
            std::cout << getres->status << std::endl;
            std::cout << getres->get_header_value("Content-Type") << std::endl;
            std::cout << getres->body << std::endl;
            //res.result= getres->body;

            nlohmann::json retJson = nlohmann::json::parse(getres->body, nullptr, false);
            if (retJson.is_discarded())
            {
                ROS_INFO("json::parse error!");
                return false;
            }
            int status = retJson["status"].get<int>();
            std::string msgStr;
            if (retJson["msg"].is_string())
                msgStr = retJson["msg"];
            if(status==100)
            {
                ROS_INFO("%s request permit , msg:%s", getStr.c_str() , msgStr.c_str());
                return true;

            }else if(status==101)
            {
                ROS_INFO("%s request prohibit , msg:%s", getStr.c_str(), msgStr.c_str());
            }else{
                ROS_INFO("request error , check your request or server response " );
                return false;
            }

        }
        else {
            std::cout << "error code: " << getres.error() << std::endl;
            return false;
        }

        rate.sleep();
    }

}

//动作请求
bool ActionRequest(httplib::Client & cli,std::string apistr,std_srvs::SetBool::Response &res)
{
    ros::Rate rate(1) ;
    httplib::Params params;
    //std::string getStr="/api/"+apistr;
    std::string getStr= "/getRequest";
    httplib::Headers headers = {
    { "Accept", "application/json" }
    };
    res.success=false;
    while(true)
    {
        if (auto getres = cli.Get(getStr,params,headers,[](uint64_t len, uint64_t total) {
            //printf("%lld / %lld bytes => %d%% complete\n",len, total,(int)(len*100/total));
            return true; 
            }))
        {
            std::cout << getres->status << std::endl;
            std::cout << getres->get_header_value("Content-Type") << std::endl;
            std::cout << getres->body << std::endl;
            //res.result= getres->body;
            nlohmann::json retJson = nlohmann::json::parse(getres->body, nullptr, false);
            if (retJson.is_discarded())
            {
                ROS_INFO("json::parse error!");
                return false;
            }
            int status = retJson["status"].get<int>();
            std::string msgStr;
            if (retJson["msg"].is_string())
                msgStr = retJson["msg"];
            if(status==100)
            {
                ROS_INFO("%s request permit , msg:%s", getStr.c_str(), msgStr.c_str());
                // 如果是place或者reclaim ，需要在动作开始执行前 发送请求
                if(apistr=="place" || apistr=="reclaim")
                {
                    bool getStart= StartActionRequst(cli,apistr);
                    if(getStart)
                    {
                        res.success=true;
                        res.message=msgStr ;
                    }else{
                        res.success=false;
                        res.message=msgStr ;
                        return true;
                    }
                }else{
                    res.success=true;
                    res.message=msgStr ;
                }
                return true;
            }else if(status==101)
            {
                ROS_INFO("%s request prohibit , msg:%s",getStr.c_str(), msgStr.c_str());
            }else{
                ROS_INFO("request error , check your request or server response " );
                res.success=false;
                return true;
            }
        }
        else {
            std::cout << "error code: " << getres.error() << std::endl;
            res.success=false;
            return true;
        }
        rate.sleep();
    }

}

bool RequestPlace(std_srvs::SetBool::Request &req,
         std_srvs::SetBool::Response &res)
{
    const char* hostaddr=host.c_str();
    httplib::Client cli(hostaddr, port);
    if(req.data)
    {
        ActionRequest(cli,"place",res);
    }
    ROS_INFO("sending back response: [%d]", res.success);
    return true;
}

bool RequestPlaced(std_srvs::SetBool::Request &req,
         std_srvs::SetBool::Response &res)
{
    const char* hostaddr=host.c_str();
    httplib::Client cli(hostaddr, port);
    if(req.data)
    {
        ActionRequest(cli,"placed",res);
    }
    ROS_INFO("sending back response: [%d]", res.success);
    return true;
}

bool RequestReclaim(std_srvs::SetBool::Request &req,
         std_srvs::SetBool::Response &res)
{
    const char* hostaddr=host.c_str();
    httplib::Client cli(hostaddr, port);
    if(req.data)
    {
        ActionRequest(cli,"reclaim",res);
    }
    ROS_INFO("sending back response: [%d]", res.success);
    return true;
}

bool RequestReclaimed(std_srvs::SetBool::Request &req,
         std_srvs::SetBool::Response &res)
{
    const char* hostaddr=host.c_str();
    httplib::Client cli(hostaddr, port);
    if(req.data)
    {
        ActionRequest(cli,"reclaimd",res);
    }
    ROS_INFO("sending back response: [%d]", res.success);
    return true;
}

int main(int argc, char **argv)
{
    setlocale(LC_ALL,"");
    //setlocale(LC_CTYPE, "zh_CN.utf8");
    ros::init(argc, argv, "http_client");
    const std::string nodeName("http_client"); 
    ros::NodeHandle nd(nodeName);

    bool gethost=nd.getParam("host", host);
	bool getport=nd.getParam("port", port);
    ROS_INFO("getparam host:%s , port:%d",host.c_str(),port);

    ros::ServiceServer service1 = nd.advertiseService("place", RequestPlace);

    ros::ServiceServer service2 = nd.advertiseService("placed", RequestPlaced);

    ros::ServiceServer service3 = nd.advertiseService("reclaim", RequestReclaim);

    ros::ServiceServer service4 = nd.advertiseService("reclaimed", RequestReclaimed);
    
    ROS_INFO("Ready to httpclient.");
    ros::spin();
    
    return 0;
}
