#!/bin/bash

case "$1" in
  start)
    echo "[*] Initialisation de l'environnement de test GNSS..."

    # 1. Créer le dossier d'enregistrement si nécessaire
    mkdir -p /home/ubuntu/Poseidon/www/webroot/record/

    # 2. Installer socat si nécessaire
    if ! command -v socat &>/dev/null; then
        echo "[+] Installation de socat..."
        sudo apt update && sudo apt install -y socat
    fi

    # 3. Stopper les processus ROS et virtuels potentiellement actifs
    echo "[*] Arrêt des processus conflictuels..."
    pkill -f gnss_zed_f9p_node && echo "[✓] gnss_zed_f9p_node arrêté" || true
    pkill -f test_gnss_zed_f9p && echo "[✓] test_gnss_zed_f9p arrêté" || true
    pkill -f roslaunch && echo "[✓] roslaunch arrêté" || true
    pkill -f socat && echo "[✓] socat arrêté" || true
    sleep 1

    # 4. Supprimer les anciens liens symboliques si présents
    rm -f /home/ubuntu/zf9p /home/ubuntu/pty /dev/ttyAMA0
    
    # 5. Démarrer socat pour créer les ports virtuels
    echo "[+] Création des ports virtuels avec socat..."
    nohup setsid socat -d -d PTY,link=/home/ubuntu/zf9p,raw,echo=0 PTY,link=/home/ubuntu/pty,raw,echo=0 > /tmp/socat.log 2>&1 &
    echo "[*] socat lancé avec PID $!"

    sleep 1

    # Résout les liens
    ZF9P_DEV=$(readlink -f /home/ubuntu/zf9p)
    PTY_DEV=$(readlink -f /home/ubuntu/pty)

    # Donne les droits à ubuntu:dialout
    sudo chown ubuntu:dialout "$ZF9P_DEV" "$PTY_DEV"
    sudo chmod g+rw "$ZF9P_DEV" "$PTY_DEV"



    sleep 1.5

    # 6. Vérifier les permissions et corriger si nécessaire
    REAL_ZF9P=$(readlink -f /home/ubuntu/zf9p)
    if [ -e "$REAL_ZF9P" ]; then
        sudo chown ubuntu:dialout "$REAL_ZF9P"
        sudo chmod 660 "$REAL_ZF9P"
        echo "[✓] Droits corrigés sur $REAL_ZF9P"
    else
        echo "[!] ERREUR : $REAL_ZF9P n'existe pas."
        exit 1
    fi

    # 7. Lier le port GNSS virtuel à /dev/ttyAMA0
    sudo ln -sf /home/ubuntu/zf9p /dev/ttyAMA0
    sudo chown -h :dialout /dev/ttyAMA0

    echo
    echo "[✓] Ports virtuels configurés avec succès"
    echo "    /home/ubuntu/zf9p   → lié à /dev/ttyAMA0 (utilisé par ROS)"
    echo "    /home/ubuntu/pty    → utilisé par les tests pour injecter les messages"

    echo "[✓] GNSS virtuel prêt."
    ;;
  stop)
    echo "[*] Arrêt des processus GNSS virtuels et nettoyage..."

    # Tuer les processus liés
    pkill -f gnss_zed_f9p_node && echo "[✓] gnss_zed_f9p_node arrêté" || true
    pkill -f test_gnss_zed_f9p && echo "[✓] test_gnss_zed_f9p arrêté" || true
    pkill -f roslaunch && echo "[✓] roslaunch arrêté" || true
    pkill -f socat && echo "[✓] socat arrêté" || true

    # Supprimer les liens symboliques
    sudo rm -f /home/ubuntu/zf9p /home/ubuntu/pty /dev/ttyAMA0 &&       echo "[✓] Liens symboliques supprimés"
    ;;
  *)
    echo "Usage: $0 {start|stop}"
    exit 1
    ;;
esac
