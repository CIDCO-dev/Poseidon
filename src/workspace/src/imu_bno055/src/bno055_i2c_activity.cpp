/* bno055_i2c_activity.cpp
 * Authors: Dheera Venkatraman <dheera@dheera.net>, Guillaume Labbe-Morissette <guillaume.morissette@cidco.ca>
 *
 * Defines a BNO055I2C Activity class, constructed with node handles
 * and which handles all ROS duties.
 */
#include <exception>
#include "imu_bno055/bno055_i2c_activity.h"

namespace imu_bno055 {

// ******** constructors ******** //

BNO055I2CActivity::BNO055I2CActivity(ros::NodeHandle &_nh, ros::NodeHandle &_nh_priv,std::string & calibrationFile) :
  nh(_nh), nh_priv(_nh_priv),calibrationFile(calibrationFile) {
    ROS_INFO("initializing");
    nh_priv.param("device", param_device, (std::string)"/dev/i2c-1");
    nh_priv.param("address", param_address, (int)BNO055_ADDRESS_A);
    nh_priv.param("frame_id", param_frame_id, (std::string)"imu");
    
    current_status.level = 0;
    current_status.name = "BNO055 IMU";
    current_status.hardware_id = "bno055_i2c";

    diagnostic_msgs::KeyValue calib_stat;
    calib_stat.key = "Calibration status";
    calib_stat.value = "";
    current_status.values.push_back(calib_stat);

    diagnostic_msgs::KeyValue selftest_result;
    selftest_result.key = "Self-test result";
    selftest_result.value = "";
    current_status.values.push_back(selftest_result);

    diagnostic_msgs::KeyValue intr_stat;
    intr_stat.key = "Interrupt status";
    intr_stat.value = "";
    current_status.values.push_back(intr_stat);

    diagnostic_msgs::KeyValue sys_clk_stat;
    sys_clk_stat.key = "System clock status";
    sys_clk_stat.value = "";
    current_status.values.push_back(sys_clk_stat);

    diagnostic_msgs::KeyValue sys_stat;
    sys_stat.key = "System status";
    sys_stat.value = "";
    current_status.values.push_back(sys_stat);

    diagnostic_msgs::KeyValue sys_err;
    sys_err.key = "System error";
    sys_err.value = "";
    current_status.values.push_back(sys_err);
}

// ******** private methods ******** //

bool BNO055I2CActivity::reset() {
    int i = 0;

    _i2c_smbus_write_byte_data(file, BNO055_OPR_MODE_ADDR, BNO055_OPERATION_MODE_CONFIG);
    ros::Duration(0.025).sleep();

    // reset
    _i2c_smbus_write_byte_data(file, BNO055_SYS_TRIGGER_ADDR, 0x20);
    ros::Duration(0.025).sleep();

    // wait for chip to come back online
    while(_i2c_smbus_read_byte_data(file, BNO055_CHIP_ID_ADDR) != BNO055_ID) {
        ros::Duration(0.010).sleep();
        if(i++ > 500) {
            ROS_ERROR_STREAM("chip did not come back online in 5 seconds after reset");
            return false;
        }
    }
    ros::Duration(0.100).sleep();


    _i2c_smbus_write_byte_data(file, BNO055_OPR_MODE_ADDR, BNO055_OPERATION_MODE_CONFIG);
    ros::Duration(0.025).sleep();

    //If a previous calibration file exists, load it
    int calibFile = open(calibrationFile.c_str(),O_RDONLY);

    if(calibFile != -1){
        IMUCalibrationRecord calibration;

        if(read(calibFile,&calibration,sizeof(IMUCalibrationRecord)) == sizeof(IMUCalibrationRecord)){
                ROS_INFO("Read calibration data from %s",calibrationFile.c_str());

		/*
		for(unsigned int i=0;i<sizeof(IMUCalibrationRecord);i+=2){
			ROS_INFO("%.2x %.2x\n",((uint8_t*)&calibration)[i],((uint8_t*)&calibration)[i+1]);
		}
		*/

		int totalBytes = 0;

		for(unsigned int i = 0;i<sizeof(IMUCalibrationRecord);i++){
			int retVal = _i2c_smbus_write_byte_data(file, BNO055_ACCEL_OFFSET_X_LSB_ADDR + i , ((unsigned char*) &calibration)[i]);
			ros::Duration(0.025).sleep();

	               	if(retVal == -1 ){
        	               	ROS_ERROR("Error while writing calibration configuration from %s",calibrationFile.c_str());
				break;
               		}
			else{
				totalBytes++;
			}
		}

		if(totalBytes == 22) ROS_INFO("Calibration loaded");

        }
	else{
		ROS_ERROR("Error while reading calibration data from %s",calibrationFile.c_str());
	}
    }
    else{
	ROS_ERROR("No calibration file found at %s",calibrationFile.c_str());
    }


    // normal power mode
    _i2c_smbus_write_byte_data(file, BNO055_PWR_MODE_ADDR, BNO055_POWER_MODE_NORMAL);
    ros::Duration(0.010).sleep();

    _i2c_smbus_write_byte_data(file, BNO055_PAGE_ID_ADDR, 0);
    _i2c_smbus_write_byte_data(file, BNO055_SYS_TRIGGER_ADDR, 0);
    ros::Duration(0.025).sleep();

    _i2c_smbus_write_byte_data(file, BNO055_OPR_MODE_ADDR, BNO055_OPERATION_MODE_NDOF);
    ros::Duration(0.025).sleep();

    /*
    for(unsigned int i=0;i<sizeof(IMUCalibrationRecord);i+=2){
	ROS_INFO("%.2x %.2x",_i2c_smbus_read_byte_data(file, BNO055_ACCEL_OFFSET_X_LSB_ADDR + i),_i2c_smbus_read_byte_data(file, BNO055_ACCEL_OFFSET_X_LSB_ADDR + i + 1));
    }
    */

    //Remap axis to ENU
    _i2c_smbus_write_byte_data(file, BNO055_AXIS_MAP_CONFIG_ADDR, 0x24);
    ros::Duration(0.010).sleep();

    _i2c_smbus_write_byte_data(file, BNO055_AXIS_MAP_SIGN_ADDR, 0x00);
    ros::Duration(0.010).sleep();


    return true;
}

// ******** public methods ******** //

bool BNO055I2CActivity::start() {
    ROS_INFO("starting");

    if(!pub_data) pub_data = nh.advertise<sensor_msgs::Imu>("data", 1);
    if(!pub_raw) pub_raw = nh.advertise<sensor_msgs::Imu>("raw", 1);
    if(!pub_mag) pub_mag = nh.advertise<sensor_msgs::MagneticField>("mag", 1);
    if(!pub_temp) pub_temp = nh.advertise<sensor_msgs::Temperature>("temp", 1);
    if(!pub_status) pub_status = nh.advertise<diagnostic_msgs::DiagnosticStatus>("status", 1);

    if(!service_calibrate) service_calibrate = nh.advertiseService(
        "calibrate",
        &BNO055I2CActivity::onServiceCalibrate,
        this
    );

    if(!service_reset) service_reset = nh.advertiseService(
        "reset",
        &BNO055I2CActivity::onServiceReset,
        this
    );

    file = open(param_device.c_str(), O_RDWR);
    if(ioctl(file, I2C_SLAVE, param_address) < 0) {
        ROS_ERROR("i2c device open failed");
        return false;
    }

    if(_i2c_smbus_read_byte_data(file, BNO055_CHIP_ID_ADDR) != BNO055_ID) {
        ROS_ERROR("incorrect chip ID");
        return false;
    }

    ROS_INFO_STREAM("rev ids:"
      << " accel:" << _i2c_smbus_read_byte_data(file, BNO055_ACCEL_REV_ID_ADDR)
      << " mag:" << _i2c_smbus_read_byte_data(file, BNO055_MAG_REV_ID_ADDR)
      << " gyro:" << _i2c_smbus_read_byte_data(file, BNO055_GYRO_REV_ID_ADDR)
      << " sw:" << _i2c_smbus_read_word_data(file, BNO055_SW_REV_ID_LSB_ADDR)
      << " bl:" << _i2c_smbus_read_byte_data(file, BNO055_BL_REV_ID_ADDR));

    if(!reset()) {
        ROS_ERROR("chip reset and setup failed");
        return false;
    }

    return true;
}

bool BNO055I2CActivity::startCalibration(){
//	_i2c_smbus_write_byte_data(file, BNO055_OPR_MODE_ADDR, BNO055_OPERATION_MODE_CONFIG);
//	ros::Duration(0.025).sleep();

	return true;
}

bool BNO055I2CActivity::spinCalibrationOnce(){
	IMURecord record;

	// can only read a length of 0x20 at a time, so do it in 2 reads
	// BNO055_LINEAR_ACCEL_DATA_X_LSB_ADDR is the start of the data block that aligns with the IMURecord struct
	if(_i2c_smbus_read_i2c_block_data(file, BNO055_ACCEL_DATA_X_LSB_ADDR, 0x20, (uint8_t*)&record) != 0x20) {
        	throw std::runtime_error("Error while reading from I2C bus");
	}

	if(_i2c_smbus_read_i2c_block_data(file, BNO055_ACCEL_DATA_X_LSB_ADDR + 0x20, 0x13, (uint8_t*)&record + 0x20) != 0x13) {
		throw std::runtime_error("Error while reading from I2C bus");
	}

	ROS_INFO("Magnetometer: %d",   record.calibration_status & 0x03);
	ROS_INFO("Accelerometer: %d", (record.calibration_status >> 2)  & 0x03);
	ROS_INFO("Gyro: %d",          (record.calibration_status >> 4)  & 0x03);
	ROS_INFO("System: %d",        (record.calibration_status >> 6)  & 0x03);

	//ROS_INFO("Calibration complete");
	if(record.calibration_status == 0xFF){
		_i2c_smbus_write_byte_data(file, BNO055_OPR_MODE_ADDR, BNO055_OPERATION_MODE_CONFIG);
		ros::Duration(0.025).sleep();

		IMUCalibrationRecord calibration;

		//Read calibration block
		int sz = sizeof(IMUCalibrationRecord);

                if(_i2c_smbus_read_i2c_block_data(file, BNO055_ACCEL_OFFSET_X_LSB_ADDR , sz, (uint8_t*)&calibration) != sz) {
                        throw std::runtime_error("Error while reading from I2C bus");
                }

		//write calibration block to file
		int outputFile = open(calibrationFile.c_str(),O_RDWR|O_CREAT|O_TRUNC);
		if(outputFile == -1){
			throw std::runtime_error("Error while opening configuration file");
		}

		if( write(outputFile,&calibration,sz) != sz){
			throw std::runtime_error("Error while writing configuration file");
		}

		close(outputFile);

		return true;
	}

	//false if calibration is not complete
        return false;
}

bool BNO055I2CActivity::spinOnce() {
    ros::spinOnce();

    ros::Time time = ros::Time::now();

    IMURecord record;

    unsigned char c = 0;

    seq++;

    // can only read a length of 0x20 at a time, so do it in 2 reads
    // BNO055_LINEAR_ACCEL_DATA_X_LSB_ADDR is the start of the data block that aligns with the IMURecord struct
    if(_i2c_smbus_read_i2c_block_data(file, BNO055_ACCEL_DATA_X_LSB_ADDR, 0x20, (uint8_t*)&record) != 0x20) {
        return false;
    }
    if(_i2c_smbus_read_i2c_block_data(file, BNO055_ACCEL_DATA_X_LSB_ADDR + 0x20, 0x13, (uint8_t*)&record + 0x20) != 0x13) {
        return false;
    }

    sensor_msgs::Imu msg_raw;
    msg_raw.header.stamp = time;
    msg_raw.header.frame_id = param_frame_id;
    msg_raw.header.seq = seq;
    msg_raw.linear_acceleration.x = (double)record.raw_linear_acceleration_x / 100.0;
    msg_raw.linear_acceleration.y = (double)record.raw_linear_acceleration_y / 100.0;
    msg_raw.linear_acceleration.z = (double)record.raw_linear_acceleration_z / 100.0;
    msg_raw.angular_velocity.x = (double)record.raw_angular_velocity_x / 900.0;
    msg_raw.angular_velocity.y = (double)record.raw_angular_velocity_y / 900.0;
    msg_raw.angular_velocity.z = (double)record.raw_angular_velocity_z / 900.0;

    sensor_msgs::MagneticField msg_mag;
    msg_mag.header.stamp = time;
    msg_mag.header.frame_id = param_frame_id;
    msg_mag.header.seq = seq;
    msg_mag.magnetic_field.x = (double)record.raw_magnetic_field_x / 16.0;
    msg_mag.magnetic_field.y = (double)record.raw_magnetic_field_y / 16.0;
    msg_mag.magnetic_field.z = (double)record.raw_magnetic_field_z / 16.0;

    sensor_msgs::Imu msg_data;
    msg_data.header.stamp = time;
    msg_data.header.frame_id = param_frame_id;
    msg_data.header.seq = seq;

    double fused_orientation_norm = std::pow(
      std::pow(record.fused_orientation_w, 2) +
      std::pow(record.fused_orientation_x, 2) +
      std::pow(record.fused_orientation_y, 2) +
      std::pow(record.fused_orientation_z, 2), 0.5);

    msg_data.orientation.w = (double)record.fused_orientation_w / fused_orientation_norm;
    msg_data.orientation.x = (double)record.fused_orientation_x / fused_orientation_norm;
    msg_data.orientation.y = (double)record.fused_orientation_y / fused_orientation_norm;
    msg_data.orientation.z = (double)record.fused_orientation_z / fused_orientation_norm;
    msg_data.linear_acceleration.x = (double)record.fused_linear_acceleration_x / 100.0;
    msg_data.linear_acceleration.y = (double)record.fused_linear_acceleration_y / 100.0;
    msg_data.linear_acceleration.z = (double)record.fused_linear_acceleration_z / 100.0;
    msg_data.angular_velocity.x = (double)record.raw_angular_velocity_x / 900.0;
    msg_data.angular_velocity.y = (double)record.raw_angular_velocity_y / 900.0;
    msg_data.angular_velocity.z = (double)record.raw_angular_velocity_z / 900.0;

    sensor_msgs::Temperature msg_temp;
    msg_temp.header.stamp = time;
    msg_temp.header.frame_id = param_frame_id;
    msg_temp.header.seq = seq;
    msg_temp.temperature = (double)record.temperature;

    pub_data.publish(msg_data);
    pub_raw.publish(msg_raw);
    pub_mag.publish(msg_mag);
    pub_temp.publish(msg_temp);

    if(seq % 50 == 0) {
        current_status.values[DIAG_CALIB_STAT].value = std::to_string(record.calibration_status);
        current_status.values[DIAG_SELFTEST_RESULT].value = std::to_string(record.self_test_result);
        current_status.values[DIAG_INTR_STAT].value = std::to_string(record.interrupt_status);
        current_status.values[DIAG_SYS_CLK_STAT].value = std::to_string(record.system_clock_status);
        current_status.values[DIAG_SYS_STAT].value = std::to_string(record.system_status);
        current_status.values[DIAG_SYS_ERR].value = std::to_string(record.system_error_code);
        pub_status.publish(current_status);
    }

    return true;    
}

bool BNO055I2CActivity::stop() {
    ROS_INFO("stopping");

    if(pub_data) pub_data.shutdown();
    if(pub_raw) pub_raw.shutdown();
    if(pub_mag) pub_mag.shutdown();
    if(pub_temp) pub_temp.shutdown();
    if(pub_status) pub_status.shutdown();

    if(service_calibrate) service_calibrate.shutdown();
    if(service_reset) service_reset.shutdown();

    return true;
}

bool BNO055I2CActivity::onServiceReset(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res) {
    if(!reset()) {
        ROS_ERROR("chip reset and setup failed");
        return false;
    }
    return true;
}

bool BNO055I2CActivity::onServiceCalibrate(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res) {
    // TODO implement this
    return true;
}

}
