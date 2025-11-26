#!/usr/bin/env bash
set -euo pipefail

# ROS 2 Jazzy install script for Ubuntu 24.04 (desktop + dev tools) and Poseidon web/hotspot deps.
# Run as root (or with sudo): sudo ./install.sh

if [[ $EUID -ne 0 ]]; then
  echo "Please run as root (sudo ./install.sh)" >&2
  exit 1
fi

echo "[1/9] Updating apt and installing base build tools..."
apt update
apt install -y --no-install-recommends \
  locales curl gnupg lsb-release ca-certificates software-properties-common \
  build-essential cmake git python3 python3-pip python3-venv \
  python3-colcon-common-extensions python3-vcstool \
  python3-rosdep python3-rosinstall-generator python3-rosinstall \
  python3-serial python3-smbus2 python3-websockets python3-libgpiod \
  libssl-dev libffi-dev gpsd gpsd-clients chrony libgps-dev

echo "[2/9] Setting UTF-8 locale..."
locale-gen en_US en_US.UTF-8
update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

echo "[3/9] Adding ROS 2 (Jazzy) apt repo..."
apt install -y --no-install-recommends curl gnupg2
curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(lsb_release -sc) main" \
  > /etc/apt/sources.list.d/ros2.list

echo "[4/9] Installing ROS 2 Jazzy (ros-base) and dev tools..."
apt update
apt install -y ros-jazzy-ros-base ros-jazzy-desktop ros-jazzy-rqt* ros-jazzy-nav-msgs ros-jazzy-gpsd-client ros-jazzy-bno055 || true

echo "[5/9] Initializing rosdep..."
rosdep init 2>/dev/null || true
sudo -u "$SUDO_USER" rosdep update

echo "[6/9] Installing Python dependencies used by the workspace (pip)..."
sudo -u "$SUDO_USER" python3 -m pip install --upgrade pip
sudo -u "$SUDO_USER" python3 -m pip install smbus2 pyserial websockets

echo "[7/9] Installing web server and websocket deps (lighttpd, websocketpp, rapidjson)..."
apt install -y lighttpd libwebsocketpp-dev rapidjson-dev
systemctl enable lighttpd.service
systemctl restart lighttpd.service
mkdir -p /opt/Poseidon/www/webroot/record
# Configure lighttpd to serve /opt/Poseidon/www/webroot
cat >/etc/lighttpd/lighttpd.conf <<'EOF_LIGHTTPD'
server.modules = (
        "mod_indexfile",
        "mod_access",
        "mod_alias",
        "mod_redirect",
        "mod_compress",
        "mod_dirlisting",
        "mod_staticfile",
)

server.document-root        = "/opt/Poseidon/www/webroot"
server.upload-dirs          = ( "/var/cache/lighttpd/uploads" )
server.errorlog             = "/var/log/lighttpd/error.log"
server.pid-file             = "/run/lighttpd.pid"
server.username             = "www-data"
server.groupname            = "www-data"
server.port                 = 80

server.http-parseopts = (
  "header-strict"           => "enable",
  "host-strict"             => "enable",
  "host-normalize"          => "enable",
  "url-normalize-unreserved"=> "enable",
  "url-normalize-required"  => "enable",
  "url-ctrls-reject"        => "enable",
  "url-path-2f-decode"      => "enable",
  "url-path-dotseg-remove"  => "enable",
)

index-file.names            = ( "index.php", "index.html" )
url.access-deny             = ( "~", ".inc" )
static-file.exclude-extensions = ( ".php", ".pl", ".fcgi" )

compress.cache-dir          = "/var/cache/lighttpd/compress/"
compress.filetype           = ( "application/javascript", "text/css", "text/html", "text/plain" )

include_shell "/usr/share/lighttpd/create-mime.conf.pl"
include "/etc/lighttpd/conf-enabled/*.conf"
EOF_LIGHTTPD
systemctl restart lighttpd.service

echo "[8/9] Installing network/hotspot helpers (NetworkManager, iptables)..."
apt install -y network-manager iptables
systemctl enable NetworkManager.service
systemctl restart NetworkManager.service
echo "Hotspot/WiFi can be configured manually with nmcli (see legacy scripts in install/stages/)."

echo "[9/9] Configuring chrony for GPS/NMEA/PPS (basic config)..."
ROOT_DIR="$(realpath "$(dirname "$0")/../..")"
CHRONY_SCRIPT="$ROOT_DIR/install/scripts/update-chrony.sh"
if [[ -x "$CHRONY_SCRIPT" ]]; then
  "$CHRONY_SCRIPT"
else
  echo "chrony update script not found at $CHRONY_SCRIPT, skipping"
fi

cat <<'EOF'
Done.

To use ROS 2 Jazzy in this shell:
  source /opt/ros/jazzy/setup.bash
  # then build your workspace:
  # cd src/ros2_ws
  # colcon build --symlink-install

EOF
