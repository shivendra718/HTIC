#!/usr/bin/env python




import socket, struct

def main():
	array = []
	#Establish connection to controller
	HOST = '172.16.101.225'
	PORT = 30002

	s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
	s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
	s.connect((HOST, PORT))

	while 1:
		#Loop forever, receive 4096 bytes of data (enough to store any packet)
		data = s.recv(4096)
		#initialise i to keep track of position in packet
		i = 0
		if data:
			#Print title to screen
			print ('*******')
			print ('UR Controller Primary Client Interface Reader') 
			print ('*******')
			#extract packet length, timestamp and packet type from start of packet and print to screen
			packlen =  (struct.unpack('!i', data[0:4]))[0]
			timestamp = (struct.unpack('!Q', data[10:18]))[0]
			# packtype = (struct.unpack('!b', data[4]))[0] 
			packtype = data[4]
			print ('packet length: ' + str(packlen))
			print ('timestamp: ' + str(timestamp))
			print ('packet type: ' + str(packtype))
			print ('*******')

			if packtype == 16:
				#if packet type is Robot State, loop until reached end of packet
				while i+5 < packlen:

					#extract length and type of message and print if desired
					msglen = (struct.unpack('!i', data[5+i:9+i]))[0] 
					# msgtype = (struct.unpack('!b', data[9+i]))[0] 
					msgtype = data[9+i]
		
					#print 'packet length: ' + str(msglen)
					#print 'message type: ' + str(msgtype)
					#print '*******'

					if msgtype == 1:
						#if message is joint data, create a list to store angles
						angle = [0]*6
						j = 0
						while j < 6:
							#cycle through joints and extract only current joint angle (double precision)  then print to screen
							#bytes 10 to 18 contain the j0 angle, each joint's data is 41 bytes long (so we skip j*41 each time)
							angle[j] = (struct.unpack('!d', data[10+i+(j*41):18+i+(j*41)]))[0]
							print ('Joint ' + str(j) + ' angle : ' + str(angle[j]))
							j = j + 1
							 
						print ('*******')
	
					elif msgtype == 4:
						#if message type is cartesian data, extract doubles for 6DOF pos of TCP and print to sc    reen
						x =  (struct.unpack('!d', data[10+i:18+i]))[0]
						y =  (struct.unpack('!d', data[18+i:26+i]))[0]
						z =  (struct.unpack('!d', data[26+i:34+i]))[0]
						rx =  (struct.unpack('!d', data[34+i:42+i]))[0]
						ry =  (struct.unpack('!d', data[42+i:50+i]))[0]
						rz =  (struct.unpack('!d', data[50+i:58+i]))[0]
						array.append([x,y,z])
						print ('X:  ' + str(x))
						print ('Y:  ' + str(y))
						print ('Z:  ' + str(z))
						print ('RX: ' + str(rx))
						print ('RY: ' + str(ry))
						print ('RZ: ' + str(rz))
						print ('*******\n')
					#increment i by the length of the message so move onto next message in packet
					i = msglen + i
					# print(array)
					print("\n")

if    __name__ == '__main__':
    import sys
    main()
	