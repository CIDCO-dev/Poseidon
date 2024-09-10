#include "state_controller/state_controller.h"

void StateController::gnssCallback(const sensor_msgs::NavSatFix& gnss){
	if( gnss.status.service > 0) { 
		stateMtx.lock();
			state.position = gnss;
			state.stamp=gnss.header.stamp;
		stateMtx.unlock();

		StateController::stateUpdated();
	}
}

void StateController::imuCallback(const sensor_msgs::Imu& imu){
	stateMtx.lock();
		state.imu = imu;
		state.stamp = imu.header.stamp;
	stateMtx.unlock();

	StateController::stateUpdated();
}

void StateController::sonarCallback(const geometry_msgs::PointStamped& sonar){
	stateMtx.lock();
		state.depth = sonar;
		state.stamp = sonar.header.stamp;
	stateMtx.unlock();

	StateController::stateUpdated();
}

void StateController::vitalsCallback(const raspberrypi_vitals_msg::sysinfo& vital){
	stateMtx.lock();
		state.vitals = vital;
		state.stamp = vital.header.stamp;
	stateMtx.unlock();

	StateController::stateUpdated();
}

bool StateController::getStateService(state_controller_msg::GetStateService::Request & req,state_controller_msg::GetStateService::Response & res){
	stateMtx.lock();
		res.state=state;
	stateMtx.unlock();

	return true;
}

void StateController::stateUpdated(){
	//TODO: kalmanize? throttle?
	stateMtx.lock();
		stateTopic.publish(state);
	stateMtx.unlock();
}


