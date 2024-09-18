sudo systemctl stop ros
sudo systemctl stop gpsd
python3 ZED-F9P_validate.py
sudo systemctl start gpsd
sudo systemctl start ros
