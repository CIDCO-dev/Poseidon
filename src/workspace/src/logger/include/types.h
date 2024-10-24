#pragma once

/* Packet types */

#define PACKET_POSITION 1
#define PACKET_ATTITUDE 2
#define PACKET_DEPTH	3
#define PACKET_LIDAR	4
#define PACKET_SPEED	5
#define PACKET_VITALS   6


/* Packet structs */
#pragma pack(1)
typedef struct {
        uint8_t   packetType;
        uint64_t  packetSize;
        uint64_t  packetTimestamp;
} PacketHeader;
#pragma pack()

#pragma pack(1)
typedef struct{
/*      GNSS Status:

        int8 STATUS_NO_FIX =  -1        # unable to fix position
        int8 STATUS_FIX =      0        # unaugmented fix
        int8 STATUS_SBAS_FIX = 1        # with satellite-based augmentation
        int8 STATUS_GBAS_FIX = 2        # with ground-based augmentation
*/
        int8_t   status;

/*
        GNSS Service

        uint16 SERVICE_GPS =     1
        uint16 SERVICE_GLONASS = 2
        uint16 SERVICE_COMPASS = 4      # includes BeiDou.
        uint16 SERVICE_GALILEO = 8
*/
        uint16_t service;

        double latitude;
        double longitude;
        double altitude;

        double covariance[9];

/*
        Covariance matrix type
        uint8 COVARIANCE_TYPE_UNKNOWN = 0
        uint8 COVARIANCE_TYPE_APPROXIMATED = 1
        uint8 COVARIANCE_TYPE_DIAGONAL_KNOWN = 2
        uint8 COVARIANCE_TYPE_KNOWN = 3 
*/
        uint8_t covarianceType;

} PositionPacket;
#pragma pack()


#pragma pack(1)
typedef struct{
        double heading;
        double pitch;
        double roll;

} AttitudePacket;
#pragma pack()

#pragma pack(1)
typedef struct {
        double depth_x;
        double depth_y;
        double depth_z;
} DepthPacket;
#pragma pack()

#pragma pack(1)
typedef struct {
	double laser_x;
	double laser_y;
	double laser_z;
} LidarPacket;
#pragma pack()

#pragma pack(1)
typedef struct {
	double speedKMH;
} SpeedPacket;
#pragma pack()

#pragma pack(1)
typedef struct {
	int nbValues;
} VitalsPacket;
#pragma pack()

#pragma pack(1)
typedef struct {
	int valueNameSize;
	std::string valueName;
	double value;
} VitalPacket;
#pragma pack()
