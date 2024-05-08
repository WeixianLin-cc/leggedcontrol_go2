#include <unitree/robot/go2/robot_state/robot_state_client.hpp>
#include <unitree/common/time/time_tool.hpp>

using namespace unitree::common;
using namespace unitree::robot;
using namespace unitree::robot::go2;

int main(int32_t argc, const char** argv)
{

    if (argc < 2)
    {
        std::cout << "Usage: robot_state_client_example [NetWorkInterface(eth0)] [ServiceName(sport_mode)] [switchValue]" << std::endl;
        exit(0);
    }

    std::string networkInterface = "eth0", serviceName = "sport_mode";
    bool switchValue;

    if (argc > 1)
    {
        networkInterface = argv[1];
    }

    if (argc > 2)
    {
        serviceName = argv[2];
        switchValue = argv[3];
    
    }
    

    std::cout << "NetWorkInterface:" << networkInterface << std::endl;
    std::cout << "Switch ServiceName:" << serviceName << std::endl; 

    ChannelFactory::Instance()->Init(0, networkInterface);

    RobotStateClient rsc;
    rsc.SetTimeout(10.0f);
    rsc.Init();

    std::string clientApiVersion = rsc.GetApiVersion();
    std::string serverApiVersion = rsc.GetServerApiVersion();

    if (clientApiVersion != serverApiVersion)
    {
        std::cout << "client and server api versions are not equal." << std::endl;
    }

    int32_t ret = rsc.ServiceSwitch(serviceName, switchValue);
    std::cout << "Call ServiceSwitch[" << serviceName << ",<"<<switchValue<<">] ret:" << ret << std::endl;

    ChannelFactory::Instance()->Release();

    return 0;
}
