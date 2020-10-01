#include "state_controller/state_controller.h"

void StateController::gnssCallback(const sensor_msgs::NavSatFix& gnss){
	if( gnss.status.service > 0) { 
		stateMtx.lock();
		memcpy(&state.position,&gnss,sizeof(gnss));
		stateMtx.unlock();

		StateController::stateUpdated();
	}
}

void StateController::imuCallback(const nav_msgs::Odometry& odom){
	stateMtx.lock();
	memcpy(&state.odom,&odom,sizeof(odom));
	stateMtx.unlock();

	StateController::stateUpdated();
}

void StateController::sonarCallback(const geometry_msgs::PointStamped& sonar){
	stateMtx.lock();
	memcpy(&state.depth,&sonar,sizeof(sonar));
	stateMtx.unlock();

        StateController::stateUpdated();
}

void StateController::vitalsCallback(const raspberrypi_vitals_msg::sysinfo& vital){
	stateMtx.lock();
        memcpy(&state.vitals,&vital,sizeof(vital));
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


