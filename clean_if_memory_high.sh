#!/bin/bash

# % mémoire utilisée
USED_MEM=$(free | awk '/Mem:/ {printf("%.0f", $3/$2 * 100)}')

THRESHOLD=80

if [ "$USED_MEM" -ge "$THRESHOLD" ]; then
    echo "[!] Mémoire utilisée: ${USED_MEM}% — action requise"

    # Tuer VS Code server
    echo "[*] Fermeture des processus vscode-server..."
    pkill -f vscode-server

    # Vider les caches
    echo "[*] Libération des caches disque..."
    sync && sudo sh -c 'echo 3 > /proc/sys/vm/drop_caches'

    # Afficher les 10 plus gros processus restants
    echo "[*] Top 10 des processus par RAM :"
    ps aux --sort=-%mem | head -n 11
else
    echo "[✓] Mémoire OK (${USED_MEM}%) — rien à faire."
fi
