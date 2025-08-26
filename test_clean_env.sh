#!/bin/bash

echo "[*] Nettoyage de l’environnement de test Poseidon..."

# 1. Tuer tous les nœuds ROS restants
echo "[*] Tentative d'arrêt via rosnode..."
rosnode kill -a

# 2. Attendre un peu
sleep 1.5

# 3. Tuer tous les processus ROS liés à Poseidon, si encore actifs
echo "[*] Suppression des processus ROS liés à Poseidon..."
pkill -f roslaunch
pkill -f rosrun
pkill -f gnss_zed_f9p
pkill -f imu_node
pkill -f websocket
pkill -f hydroball
pkill -f state_controller
pkill -f sonar
pkill -f logger
pkill -f diagnostics
pkill -f gps
pkill -f i2c_controller

# 4. Arrêt de socat si présent
pkill -f socat && echo "[✓] socat arrêté"

# 5. Supprimer les liens série virtuels
rm -f /home/ubuntu/zf9p /home/ubuntu/pty /dev/ttyAMA0 && echo "[✓] Liens virtuels supprimés"

# 6. Affichage final
echo "[✓] Environnement nettoyé. Prêt pour les tests."
