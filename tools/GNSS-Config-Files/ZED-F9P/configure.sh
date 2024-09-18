sudo systemctl stop ros
sudo systemctl stop gpsd
python3 ZED-F9P_config.py
sudo systemctl start gpsd
sudo systemctl start ros
