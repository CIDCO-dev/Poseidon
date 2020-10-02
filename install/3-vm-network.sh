#!/bin/sh

echo "[+] Installing web server"
sudo apt install lighttpd -y | tee -a log.txt

sudo systemctl start lighttpd.service | tee -a log.txt
sudo systemctl enable lighttpd.service | tee -a log.txt

echo "[+] Installing websocket and JSON libraries"
sudo apt install libwebsocketpp-dev rapidjson-dev

mkdir -p /home/jordan/Poseidon/www/webroot/record

sudo bash -c 'cat << EOF2 > /etc/lighttpd/lighttpd.conf
server.modules = (
	"mod_access",
	"mod_alias",
	"mod_compress",
 	"mod_redirect",
)

server.document-root        = "/home/jordan/Poseidon/www/webroot"
server.upload-dirs          = ( "/var/cache/lighttpd/uploads" )
server.errorlog             = "/var/log/lighttpd/error.log"
server.pid-file             = "/var/run/lighttpd.pid"
server.username             = "www-data"
server.groupname            = "www-data"
server.port                 = 80


index-file.names            = ( "index.php", "index.html", "index.lighttpd.html" )
url.access-deny             = ( "~", ".inc" )
static-file.exclude-extensions = ( ".php", ".pl", ".fcgi" )

compress.cache-dir          = "/var/cache/lighttpd/compress/"
compress.filetype           = ( "application/javascript", "text/css", "text/html", "text/plain" )

# default listening port for IPv6 falls back to the IPv4 port
## Use ipv6 if available
#include_shell "/usr/share/lighttpd/use-ipv6.pl " + server.port
include_shell "/usr/share/lighttpd/create-mime.assign.pl"
include_shell "/usr/share/lighttpd/include-conf-enabled.pl"
EOF2'
sudo systemctl restart lighttpd.service
