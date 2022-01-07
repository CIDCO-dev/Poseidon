import socket
import sys
import nmeasim
 
# Set up a TCP/IP server
tcp_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
 
# Bind the socket to server address and port 
server_address = ('127.0.0.1', 8000)
tcp_socket.bind(server_address)
 
# Listen on port
tcp_socket.listen(1)
 
while True:
	print('waiting for a connection')
	connection, client_address = tcp_socket.accept()
	try:
		print ('client connected:', client_address)
		while True:
			#data = connection.recv(16)
			#print('received "%s"' % data)
			data = 'GPVTG,82.0,T,77.7,M,2.4,N,20,K,S*3A'
			data = bytes(data,'UTF-8')
			if data:
				connection.sendall(data)
			else:
				break
	finally:
		connection.close()
"""	
				#GPVTG,82.0,T,77.7,M,2.4,N,4.4,K,S*3A
				data = 'GPVTG,82.0,T,77.7,M,2.4,N,20,K,S*3A'
				data = bytes(data,'UTF-8')
				tcp_socket.sendall(data)
"""
