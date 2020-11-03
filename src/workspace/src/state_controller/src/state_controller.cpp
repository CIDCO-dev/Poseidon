#include "state_controller/state_controller.h"

void StateController::gnssCallback(const sensor_msgs::NavSatFix& gnss){
	if( gnss.status.service > 0) { 
		stateMtx.lock();
		state.position = gnss;
		stateMtx.unlock();

		StateController::stateUpdated();
	}
}

void StateController::imuCallback(const sensor_msgs::Imu& imu){
	stateMtx.lock();
	state.imu = imu;
	stateMtx.unlock();

	StateController::stateUpdated();
}

void StateController::sonarCallback(const geometry_msgs::PointStamped& sonar){
	stateMtx.lock();
    state.depth = sonar;
	stateMtx.unlock();

	StateController::stateUpdated();
}

void StateController::vitalsCallback(const raspberrypi_vitals_msg::sysinfo& vital){
	stateMtx.lock();
    state.vitals = vital;
	stateMtx.unlock();

	StateController::stateUpdated();
}

bool StateController::getStateService(state_controller_msg::GetStateService::Request & req,state_controller_msg::GetStateService::Response & res){
	stateMtx.lock();
	res.state=state; //deep copy
	stateMtx.unlock();

	return true;
}

void StateController::stateUpdated(){
	//TODO: kalmanize? throttle?
	stateMtx.lock();
        stateTopic.publish(state);
	stateMtx.unlock();
}


