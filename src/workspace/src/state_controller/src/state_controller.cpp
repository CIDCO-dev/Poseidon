#include "state_controller/state_controller.h"



		void StateController::gnssCallback(const sensor_msgs::NavSatFix& gnss){
                    if( gnss.status.service > 0) { 
			memcpy(&state.position,&gnss,sizeof(gnss));
			StateController::stateUpdated();
			}
		}

		void StateController::imuCallback(const sensor_msgs::Imu& imu){
                    memcpy(&state.attitude,&imu,sizeof(imu));
                    StateController::stateUpdated();
		}

		void StateController::sonarCallback(const geometry_msgs::PointStamped& sonar){
                    memcpy(&state.depth,&sonar,sizeof(sonar));
                    StateController::stateUpdated();
		}

		void StateController::vitalsCallback(const raspberrypi_vitals_msg::sysinfo& vital){
                    memcpy(&state.vitals,&vital,sizeof(vital));
                    StateController::stateUpdated();
		}


		//TODO: add other sensor callbacks

		void StateController::stateUpdated(){
                    //TODO: kalmanize?
                    stateTopic.publish(state);
		}

uint64_t StateController::buildTimeStamp(int sec, int nsec){
  std::ostringstream os;
  os << sec;
  os << nsec;
  std::string ntime = os.str();

  uint64_t timestamp;
  std::istringstream iss(ntime.c_str());
  iss >> timestamp;

  return timestamp;
}
