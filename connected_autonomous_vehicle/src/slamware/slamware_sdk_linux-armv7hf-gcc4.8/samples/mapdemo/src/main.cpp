// Slamware
#include <iostream>
#include <regex>
#include <boost/thread.hpp>
#include <boost/chrono.hpp>
#include <rpos/robot_platforms/slamware_core_platform.h>
#include <rpos/features/location_provider/map.h>

// Socket Server
#include <unistd.h> 
#include <stdio.h> 
#include <sys/socket.h>
#include <stdlib.h> 
#include <netinet/in.h> 
#include <string.h> 
#include <fcntl.h>
#include <iostream>
#include <sys/stat.h>
#include <sys/types.h>
#include <string>

// Protobuff
#include <fstream>
//#include "lidarmessage.pb.h"

#define PORT 65432 

using namespace rpos::robot_platforms;
using namespace rpos::features;
using namespace rpos::features::location_provider;
using namespace rpos::system::types;
//using namespace google::protobuf;

std::string ipaddress = "192.168.11.1";

class Slamware {
    private:
        SlamwareCorePlatform sdp;
	float x;
	float y;
	float yaw;

    public:
        //moneomonelidar::lidarmessage lidarMessage;
        bool connectToSlamware();
	bool checkForChange();
	std::stringstream getData();
};

bool Slamware::connectToSlamware(){
    try {
        sdp = SlamwareCorePlatform::connect(ipaddress, 1445);
        std::cout <<"SDK Version: " << sdp.getSDKVersion() << std::endl;
        std::cout <<"SDP Version: " << sdp.getSDPVersion() << std::endl;
        std::cout <<"Bartter Status: " << sdp.getBatteryIsCharging() << std::endl;
        std::cout <<"Barttery Percetage: " << sdp.getBatteryPercentage() << std::endl;
        std::cout <<"Power Status: " << sdp.getDCIsConnected() << std::endl;
    } catch(ConnectionTimeOutException& e) {
        std::cout <<e.what() << std::endl;
        return 0;
    } catch(ConnectionFailException& e) {
        std::cout <<e.what() << std::endl;
        return 0;
    }
    std::cout <<"Connection Successfully" << std::endl;

    rpos::core::Pose pose = sdp.getPose();
    std::cout << "Robot Pose: " << std::endl;
    std::cout << "x: " << pose.x() << ", ";
    std::cout << "y: " << pose.y() << ", ";
    std::cout << "yaw: " << pose.yaw() << std::endl;

    return 1;
}

bool Slamware::checkForChange(){
    rpos::core::Pose pose = sdp.getPose();
    if(pose.x() != x or pose.y() != y or pose.yaw() != yaw){
        x = pose.x();
	y = pose.y();
	yaw = pose.yaw();
	return 1;
    } else {
	return 0;
    }
}

/*void Slamware::getDataProtobuf(){
    rpos::core::Pose pose = sdp.getPose();
    
    // Use protobuf to serialize this nicely
    int counter = 0;
    lidarMessage.set_x(pose.x());
    lidarMessage.set_y(pose.y());
    lidarMessage.set_yaw(pose.yaw());

    rpos::features::system_resource::LaserScan laser_scan = sdp.getLaserScan();
    std::vector<rpos::core::LaserPoint> laser_points =laser_scan.getLaserPoints();       
    for (std::vector<rpos::core::LaserPoint>::iterator it = laser_points.begin(); it!= laser_points.end(); ++it){
        //std::cout << "Angle: " << it->angle() << "; Distance: " << it->distance() << "; is Valid: " << it->valid() << std::endl;
        lidarMessage.add_valid(it->valid());
        lidarMessage.add_angle(it->angle());
        lidarMessage.add_distance(it->distance());
        counter++;
    }

    lidarMessage.set_size(counter);

    std::cout << "counter: " << counter << std::endl;
}*/

std::stringstream Slamware::getData(){
    rpos::core::Pose pose = sdp.getPose();
    
    // This is way uglier than protobuf
    int counter = 0;
    
    std::stringstream buffer;
    buffer << pose.x() << "," << pose.y() << "," << pose.yaw() << "\n";

    rpos::features::system_resource::LaserScan laser_scan = sdp.getLaserScan();
    std::vector<rpos::core::LaserPoint> laser_points =laser_scan.getLaserPoints();       
    for (std::vector<rpos::core::LaserPoint>::iterator it = laser_points.begin(); it!= laser_points.end(); ++it){
        //std::cout << "Angle: " << it->angle() << "; Distance: " << it->distance() << "; is Valid: " << it->valid() << std::endl;
        buffer << it->valid() << "," << it->angle() << "," << it->distance() << "\n";
        counter++;
    }

    buffer << std::endl;

    std::cout << "counter: " << counter << std::endl;

    return buffer;
}


/* TODO: Add in ability to load and save maps, see below code
    // Save map
    rpos::robot_platforms::objects::CompositeMap composite_map = sdp.getCompositeMap();
    rpos::robot_platforms::objects::CompositeMapWriter composite_map_writer;
    std::string error_message;
    bool result = composite_map_writer.saveFile(error_message, "mapsave", composite_map);

    // Load map
    rpos::robot_platforms::objects::CompositeMapReader composite_map_reader;
    std::string error_message;
    boost::shared_ptr<CompositeMap> composite_map(composite_map_reader.loadFile(error_message, "mapsave"));
    if (composite_map) {
        sdp.setCompositeMap((*composite_map), pose);  
        return true;
    }
    return false;

    //recover localization by giving an rectangle area; (0,0,0,0) represents the entire map area.
    action = sdp.recoverLocalization(rpos::core::RectangleF(0,0,0,0));
 
    while(true)
    {
        switch (action.getStatus()){
	    case rpos::core::ActionStatusError:
	    std::cout << "Action Failed: " << action.getReason() << std::endl;
	    break;
	    case rpos::core::ActionStatusRunning:
	    std::cout <<"Current status: Running" << std::endl;
	    break;
	    case rpos::core::ActionStatusFinished:
	    std::cout <<"Current status: Finished" << std::endl;
	    break;
	    default :
	    std::cout <<"Status Unknown" << std::endl;
	    break;
	}
    }*/



int main(int argc, const char * argv[])
{

    // Create socket server
    int server_fd, new_socket, valread; 
    struct sockaddr_in address; 
    int opt = 1; 
    int addrlen = sizeof(address); 
    char buffer[1] = {0};

    // Establish Slamware Connection
    Slamware slamwareObj;
    slamwareObj.connectToSlamware();
    /*char* response = slamwareObj.getData();
    int_to_bytes counter;
    counter.iBuff[0] = response[12];
    counter.iBuff[1] = response[13];
    counter.iBuff[2] = response[14];
    counter.iBuff[3] = response[15];
    std::cout << "counter: " << counter.i << std::endl;*/
       
    int pipefromcfd;
    char * fromc = "/home/jetson/Projects/slamware/fifo_queues/fifopipefromc";
    int pipetocfd;
    char * toc = "/home/jetson/Projects/slamware/fifo_queues/fifopipetoc";
    char buf[1];

    /* create the FIFO (named pipe) */
    //mkfifo(fromc, 0666);
    //mkfifo(toc, 0666);
    bool loopVar = 1;
    while(loopVar)
    {
	    pipetocfd = open(toc, O_RDONLY);
	    valread = read(pipetocfd, buf, 1);
	    close(pipetocfd);
	    std::cout << buf << valread << std::endl;
	    if(valread > 0){
		if(buf[0] == 'S'){
		    //std::cout << "Received:"<< buf << valread << std::endl;
		    char *msg;
		    msg="A";
		    pipefromcfd = open(fromc, O_WRONLY);
		    write(pipefromcfd, msg, strlen(msg)+1);
		    close(pipefromcfd);
		    // We are connected
                    loopVar = 0;
		}
	    }
	    memset(buf, 0, sizeof buf);
    }

    // Wait for a second
    usleep(1000000);

    while(true)
    {
        try {
	    if( slamwareObj.checkForChange() ){
		//std::cout << "Responding" << std::endl;
                std::stringstream returned = slamwareObj.getData();
                //std::string* output;
                //lidarMessage->SerializeToString(output)
                //std::fstream output(fromc, std::ios::out | std::ios::trunc | std::ios::binary);
		//pipefromcfd = open(fromc, O_WRONLY);
                pipefromcfd = open(fromc, O_WRONLY);
		write(pipefromcfd, returned.str().c_str(), returned.str().size()+1);
                close(pipefromcfd);
                //slamwareObj.lidarMessage.SerializeToOstream(&output);
                //write(pipefromcfd, "dupe", 4+1);
		//output.close();
	        //std::cout << "Responded" << returned.str() << std::endl;
	    }
        } catch(ConnectionTimeOutException& e) {
            std::cout <<e.what() << std::endl;
            return 1;
        }
	// Wait for a 1 ms
        usleep(1000);
    }

    /* close the FIFO queue */
    close(pipetocfd);
    close(pipefromcfd);

    /* remove the FIFO queue */
    unlink(fromc);
    unlink(toc);

    return 0;
}
