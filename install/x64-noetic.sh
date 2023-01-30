#!/bin/sh


echo "Moving software to compile"
sudo chmod 777 /opt
cp -r ../../Poseidon/ /opt/ 


/opt/Poseidon/install/stages/1-base-ros-noetic.sh

/opt/Poseidon/install/stages/2-x64.sh

/opt/Poseidon/install/stages/3-network.sh

/opt/Poseidon/install/stages/4-x64.sh

/opt/Poseidon/install/stages/5-finalize.sh

/opt/Poseidon/install/stages/6-echoboat.sh

sudo /opt/Poseidon/install/stages/install_geographiclib_dataset.sh
