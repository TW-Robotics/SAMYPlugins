
#include "plugin.h"
#include "robot.h"

#include <signal.h>
#include <signals.h>
#include <boost/signals2.hpp>
#include "namespace_crcl_generated.h"
#include "types_crcl_generated_handling.h"
#include <memory>

UA_Boolean running = true;
static void stopHandler(int sign) {
    //UA_LOG_INFO(UA_Log_Stdout, UA_LOGCATEGORY_SERVER, "received ctrl-c");
    running = false;
}

int main(int argc, char** argv)
{
    signal(SIGINT, stopHandler);
    signal(SIGTERM, stopHandler);
    printf("C++: Starting Plugin\n");

    std::string address; // address of robot
    std::string samyCoreAddress;
    std::string samyCorePort = "4840";
    std::string robotName;

    if (argc == 4){
        samyCoreAddress = argv[1];
        robotName = argv[2];
        address = argv[3];
    } else {
        std::cout << "To few arguments: \n"
                     "\"address of SAMYCore\" \"name of Robot in SAMYCore\" "
                     "\"address(ip) of robot\"" << std::endl;
        return -1;
    }

    Signals signals; // Struct of all available signals (all CRCL-commands, signals for skill methods)
    Plugin plugin(samyCoreAddress, samyCorePort, &signals);
    plugin.running = &running;

    // Creating the robot object
    std::cout << "Connecting to Robot..." << std::endl;
    std::shared_ptr<Robot> robot(new Robot(address, &signals));
    //robot.get()->ConnectToDoosan(address);
    std::cout << "Connected to Robot" << std::endl;

    UA_StatusCode retval;
    retval = plugin.InitPlugin(robotName);
    printf("Starting RunClient.\n");
    retval = plugin.RunClient(1000);
    printf("RunClient has returned.\n");

    return 0;
}
