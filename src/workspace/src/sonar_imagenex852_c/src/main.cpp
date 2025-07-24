#include "ros/ros.h"
#include "geometry_msgs/PointStamped.h"
#include <fstream>
#include <unistd.h>
#include <cstdio>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <termios.h>
#include <errno.h>
#include <mutex>
#include <thread>

#include "setting_msg/Setting.h"
#include "setting_msg/ConfigurationService.h"
#include "binary_stream_msg/Stream.h"

#pragma pack(push,1)
typedef struct{
    uint8_t magic[2];
    uint8_t headId;
    uint8_t range;
    uint8_t reserved[2];
    uint8_t masterSlave;
    uint8_t reserved2;
    uint8_t startGain;
    uint8_t reserved3;
    uint8_t absorption; // 20 = 0.2db 675kHz
    uint8_t reserved4[3];
    uint8_t pulseLength; //1-255 -> 1us to 255us
    uint8_t profileMinimumRange; //Min range in meters / 10
    uint8_t reserved5[2];
    uint8_t triggerControl;
    uint8_t dataPoints;
    uint8_t reserved6[2];
    uint8_t profile;
    uint8_t reserved7;
    uint8_t switchDelay;
    uint8_t frequency;
    uint8_t terminationByte;
} Imagenex852SwitchDataCommand;
#pragma pack(pop)

#pragma pack(push,1)
typedef struct{
    uint8_t magic[3];       // 'I','M'/'G'/'P','X'
    uint8_t headId;
    uint8_t serialStatus;
    uint8_t reserved[2];
    uint8_t range;
    uint8_t profileRange[2];
    uint8_t dataBytes[2];
} Imagenex852ReturnDataHeader;
#pragma pack(pop)

class Imagenex852{
public:
    Imagenex852(const std::string& devicePath)
    : devicePath(devicePath)
    {
        sonarTopic        = node.advertise<geometry_msgs::PointStamped>("depth", 1000);
        sonarBinStreamTopic = node.advertise<binary_stream_msg::Stream>("sonar_bin_stream", 1000);
        sonarTopicEnu     = node.advertise<geometry_msgs::PointStamped>("depth_enu", 1000);

        configSubscriber  = node.subscribe("configuration", 1000, &Imagenex852::configurationChange, this);
        configurationClient = node.serviceClient<setting_msg::ConfigurationService>("get_configuration");
        configurationClient.waitForExistence();

        ros::NodeHandle pnh("~");
        std::string trig;
        pnh.param<std::string>("trigger_mode", trig, std::string("auto"));
        triggerAuto_ = (trig != "manual");               // auto sinon manual
        pnh.param("manual_ping_rate", manualPingHz_, 2.0); // 2 Hz par défaut
        if(manualPingHz_ < 0.1) manualPingHz_ = 0.1;

        // (optionnel) data_points param
        // pnh.param("data_points", dataPoints, (uint8_t)0);

        ROS_INFO("Fetching sonar configuration...");
        getConfiguration();
    }

    ~Imagenex852(){
        if(deviceFile >= 0) close(deviceFile);
    }

    void set_dataPoints(uint8_t _dataPoints){
        this->dataPoints = _dataPoints;
    }

    void getConfiguration(){
        const char *configKeys[] = {"sonarStartGain","sonarRange","sonarAbsorbtion","sonarPulseLength"};
        uint8_t * valuePtrs[]    = {&sonarStartGain,&sonarRange,&sonarAbsorbtion,&sonarPulseLength};

        for(int i=0;i<4;i++){
            std::string valueString = getConfigValue(configKeys[i]);
            setConfigValue(valueString, valuePtrs[i]);
        }
    }

    std::string getConfigValue(std::string key){
        setting_msg::ConfigurationService srv;
        srv.request.key = key;
        if(configurationClient.call(srv)){
            return srv.response.value;
        }
        return "";
    }

    void setConfigValue(const std::string & valStr,uint8_t * val){
        std::lock_guard<std::mutex> lock(mtx);
        sscanf(valStr.c_str(),"%hhu",val);
        this->configChanged = true;
    }

    void configurationChange(const setting_msg::Setting & setting){
        if(setting.key == "sonarStartGain"){
            setConfigValue(setting.value,&sonarStartGain);
        }else if(setting.key == "sonarRange"){
            setConfigValue(setting.value,&sonarRange);
        }else if(setting.key == "sonarAbsorbtion"){
            setConfigValue(setting.value,&sonarAbsorbtion);
        }else if(setting.key == "sonarPulseLength"){
            setConfigValue(setting.value,&sonarPulseLength);
        }
    }

    void run(){
        // open serial port
        deviceFile = open(devicePath.c_str(),O_RDWR);
        if(deviceFile < 0){
            ROS_ERROR("Error while opening file %s : %s",devicePath.c_str(),strerror(errno));
            throw std::invalid_argument(strerror(errno));
        }

        // get serial config
        struct termios tty;
        memset(&tty, 0, sizeof tty);

        if(tcgetattr(deviceFile, &tty) !=0) {
            ROS_ERROR("Error while fetching serial port configuration on %s : %s",devicePath.c_str(),strerror(errno));
            throw std::invalid_argument(strerror(errno));
        }

        // set serial config
        tty.c_cflag &= ~PARENB;
        tty.c_cflag &= ~CSTOPB;
        tty.c_cflag |= CS8;
        tty.c_cflag &= ~CRTSCTS;
        tty.c_cflag |= CREAD | CLOCAL;

        tty.c_lflag &= ~ICANON;
        tty.c_lflag &= ~ECHO;
        tty.c_lflag &= ~ECHOE;
        tty.c_lflag &= ~ECHONL;
        tty.c_lflag &= ~ISIG;

        tty.c_iflag &= ~(IXON | IXOFF | IXANY);
        tty.c_iflag &= ~(IGNBRK|BRKINT|PARMRK|ISTRIP|INLCR|IGNCR|ICRNL);
        tty.c_oflag &= ~OPOST;
        tty.c_oflag &= ~ONLCR;

        tty.c_cc[VTIME] = 250; // 25 s
        tty.c_cc[VMIN]  = 0;

        cfsetispeed(&tty, B115200);
        cfsetospeed(&tty, B115200);

        if(tcsetattr(deviceFile,TCSANOW,&tty) != 0){
            ROS_ERROR("Error while configuring serial port %s : %s",devicePath.c_str(),strerror(errno));
            throw std::invalid_argument(strerror(errno));
        }

        ROS_INFO("Sonar file opened on %s",devicePath.c_str());

        ros::Rate error_rate(1);

        // Premier envoi
        send_command();
        lastPingTime_ = ros::Time::now();

        // Boucle
        while(ros::ok()){
            try{
                // Mode manuel : renvoyer la commande périodiquement pour pinger
                if(!triggerAuto_){
                    ros::Time now = ros::Time::now();
                    if((now - lastPingTime_).toSec() >= (1.0 / manualPingHz_)){
                        send_command();
                        lastPingTime_ = now;
                    }
                }

                uint8_t read_buf[1];
                uint8_t packetType=0;

                // read sync 'I'
                if(serialRead((uint8_t*)&read_buf, sizeof(read_buf)) == 1){

                    // si config a changé, renvoyer la commande immédiatement
                    if(this->configChanged){
                        send_command();
                        std::lock_guard<std::mutex> lock(mtx);
                        this->configChanged = false;
                    }

                    if(read_buf[0] == 'I'){              // 73
                        // next char: 'M' / 'G' / 'P'
                        if(serialRead((uint8_t*)&read_buf, sizeof(read_buf)) == 1){
                            packetType = read_buf[0];

                            // 'X'
                            if(serialRead((uint8_t*)&read_buf, sizeof(read_buf)) == 1){
                                if(read_buf[0] == 0x58){
                                    Imagenex852ReturnDataHeader hdr;
                                    // on a déjà lu 3 chars, on lit le reste (taille = 9)
                                    if(serialRead(((uint8_t*)&hdr)+3, sizeof(Imagenex852ReturnDataHeader)-3) == 9){
                                        hdr.magic[0] = 'I';
                                        hdr.magic[1] = packetType;
                                        hdr.magic[2] = 'X';
                                        process_data(hdr);
                                    }
                                }
                            }
                        }
                    }
                    // sinon, octet bruité, on ignore
                }
                // else => timeout, rien reçu, on continue

            }catch(std::exception & e){
                error_rate.sleep();
            }

            ros::spinOnce();
        }
    }

    int serialRead(uint8_t * buf,unsigned int sz){
        unsigned int totalRead = 0;
        while(totalRead < sz){
            int bytesRead = read(deviceFile, &(buf[totalRead]), sz - totalRead);

            if(bytesRead > 0){
                totalRead += bytesRead;
            }else if(bytesRead == 0){
                // EOF
                return 0;
            }else{
                return -1;
            }
        }
        return totalRead;
    }

    void send_command(){
        usleep(3000);

        Imagenex852SwitchDataCommand cmd;
        memset(&cmd,0,sizeof(Imagenex852SwitchDataCommand));

        {
            std::lock_guard<std::mutex> lock(mtx);
            cmd.range       = sonarRange;
            cmd.startGain   = sonarStartGain;
            cmd.absorption  = sonarAbsorbtion;   // 20 = 0.2db 675kHz
            cmd.pulseLength = sonarPulseLength;  // 1-255 -> 1us to 255us
        }

        cmd.magic[0]  = 0xFE;
        cmd.magic[1]  = 0x44;
        cmd.headId    = 0x11;
        cmd.masterSlave = 0x43;  // Slave + Transmit + Send Data
        cmd.profileMinimumRange = 0;
        // Trigger
        // 0x07 = auto (Enable=1, Mode=1, Edge=POS); 0x03 = manual (Enable=1, Mode=0, Edge=POS)
        cmd.triggerControl = triggerAuto_ ? 0x07 : 0x03;
        // Data points -> IMX=25, IGX=50, 0 = IPX (pas de profil)
        cmd.dataPoints  = (this->dataPoints > 0)? this->dataPoints : 0;
        cmd.profile     = (this->dataPoints > 0)? 0 : 1; // si pas de dataPoints -> profile=1
        cmd.switchDelay = 0;
        cmd.frequency   = 0;
        cmd.terminationByte = 0xFD;

        unsigned int nbBytes;
        if( (nbBytes = write(deviceFile,&cmd,sizeof(Imagenex852SwitchDataCommand))) != 27){
            ROS_ERROR("Cannot write switch data command (%d bytes written)",nbBytes);
            throw std::system_error();
        }

        usleep(2300); // Doc: petit délai
    }

    void process_data(const Imagenex852ReturnDataHeader & hdr){
        // config changed?
        if(hdr.range == sonarRange){
            this->configChanged = false;
        }else{
            this->configChanged = true;
        }

        int dataSize = 0;
        if(hdr.magic[1] == 'M'){
            dataSize = 252;
        }else if(hdr.magic[1] == 'G'){
            dataSize = 500;
        }else if(hdr.magic[1] == 'P'){
            // no data points
        }else{
            ROS_ERROR("Unknown Packet type: %x", hdr.magic[1]);
        }

        geometry_msgs::PointStamped msg;
        msg.header.seq = sequenceNumber++;
        msg.header.frame_id = "sonar";

        // Verify capabilities
        if(!(hdr.serialStatus & 0x01)){
            ROS_ERROR("Echosounder not detected");
        }
        if(!(hdr.serialStatus & 0x04)){
            ROS_ERROR("Automatic trigger mode not supported. Pings may be unsynchronized");
        }
        if(hdr.serialStatus & 0x80){
            ROS_ERROR("Character overrun detected");
        }

        msg.header.stamp = ros::Time::now();
        msg.header.stamp.nsec = delayNanoseconds;

        std::vector<uint8_t> binaryStreamMsg;
        binary_stream_msg::Stream stream;

        if(dataSize > 0){
            std::vector<uint8_t> echoData(dataSize, 0);
            int nbBytes = serialRead(echoData.data(), dataSize);
            if(nbBytes != dataSize){
                ROS_ERROR("Could not read datapoints ( %d bytes read)",nbBytes);
                throw std::system_error();
            }

            const uint8_t* bytePtr = reinterpret_cast<const uint8_t*>(&hdr);
            binaryStreamMsg.insert(binaryStreamMsg.end(), bytePtr, bytePtr + sizeof(Imagenex852ReturnDataHeader));
            binaryStreamMsg.insert(binaryStreamMsg.end(), echoData.begin(), echoData.end());
        }

        uint8_t terminationCharacter;
        do{
            serialRead(&terminationCharacter,sizeof(uint8_t));
        }while(terminationCharacter != 0xFC);

        // Depth calculation
        uint16_t profileHigh = (hdr.profileRange[1] & 0x7E) >> 1;
        uint16_t profileLow  = ((hdr.profileRange[1] & 0x01) << 7) | (hdr.profileRange[0] & 0x7F);
        uint16_t depthCentimeters = (profileHigh << 8) | profileLow;

        msg.point.z = (double)depthCentimeters / 100.0;
        sonarTopic.publish(msg);

        // ENU
        msg.header.frame_id = "sonar_enu";
        msg.point.z = -msg.point.z;
        sonarTopicEnu.publish(msg);

        if(binaryStreamMsg.size() > sizeof(Imagenex852ReturnDataHeader)){
            binaryStreamMsg.push_back(terminationCharacter);
            stream.vector_length = binaryStreamMsg.size();
            stream.stream = binaryStreamMsg;
            sonarBinStreamTopic.publish(stream);
        }
    }

private:
    std::mutex mtx;
    uint8_t sonarStartGain   = 0x06;
    uint8_t sonarRange       = 5;
    uint8_t sonarAbsorbtion  = 0x14; // 20 = 0.2db 675kHz
    uint8_t sonarPulseLength = 150;
    uint8_t dataPoints       = 0;

    bool    triggerAuto_     = true;   // true: auto, false: manual
    double  manualPingHz_    = 2.0;
    ros::Time lastPingTime_;

    ros::NodeHandle node;
    ros::Publisher  sonarTopic;
    ros::Publisher  sonarTopicEnu;
    ros::Publisher  sonarBinStreamTopic;
    ros::ServiceClient configurationClient;
    ros::Subscriber   configSubscriber;

    std::string devicePath;
    int deviceFile = -1;

    uint32_t delayNanoseconds = 0; // FIXME: rosparam
    uint32_t sequenceNumber    = 0;

    bool configChanged = false;
};

int main(int argc,char ** argv){
    ros::init(argc,argv,"sonar_imagenex852");

    try{
        Imagenex852 sonar("/dev/sonar");

        // dataPoints via argument classique (optionnel)
        if(argc == 2){
            try{
                uint8_t dp = (uint8_t)std::stoi(argv[1]);
                sonar.set_dataPoints(dp);
            }catch(std::exception &e){
                ROS_ERROR("Error: %s",e.what());
            }
        }

        std::thread t(std::bind(&Imagenex852::run,&sonar));
        ros::spin();

        if (t.joinable()) {
            t.join();
        }

    }catch(std::exception &e){
        ROS_ERROR("Error: %s",e.what());
        return -1;
    }

    return 0;
}
