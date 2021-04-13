#!/usr/bin/python3
 
# Copyright (C) 2006-2021 Istituto Italiano di Tecnologia (IIT)
# Copyright (C) 2006-2010 RobotCub Consortium
# All rights reserved.
#
# This software may be modified and distributed under the terms of the
# BSD-3-Clause license. See the accompanying LICENSE file for details.
 
import socket
import re
import sys
import codecs
 
if len(sys.argv)!=3:
    print('Call as:\n  %s /port/to/write/to \"message to send\"',sys.argv[0])
    exit(1)
 
# try:
#     import find_name_server
#     name_server = find_name_server.find_name_server()
#     print ("Nameserver is here:", name_server)
# except:
name_server = ('192.168.1.36',10000)
print ("Assuming nameserver is here:", name_server)
 
port_name = sys.argv[1]
message = sys.argv[2]
 
def get_addr(s):
    m = re.match("registration name [^ ]+ ip ([^ ]+) port ([0-9]+) type tcp",s)
    return (m.group(1),int(m.group(2))) if m else None
 
# get a single line of text from a socket
def getline(sock):
    result = ""
    print(sock.recv(1024))
    # while result.find('\n')==-1:
    #     result = result + sock.recv(1024)
    # result = re.sub('[\r\n].*','',result)
    # print(result)
    # return result
 
# send a message and expect a reply
def comm(addr,message):
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    sock.connect(addr)
    sock.send(b'\x59\x41\x61\x1E\x00\x00\x52\x50\n')
    sock.send(b'\x00\x00\x00\x00\x00\x00\x00\x01\n')
    sock.send(b'neee\n')
    sock.send(b'\x01\x01\xFF\xFF\xFF\xFF\xFF\xFF\xFF\xFF\n')
    sock.send(b'\x01\x01\xFF\xFF\n')
    sock.send(b'\x00\x00\x00\x00\n')
    # print(result)
    sock.close()
    return
# ask name server for location
comm(name_server,"query %s"%port_name)
# query = get_addr(comm(name_server,"query %s"%port_name))
# print("Talking to", port_name, "here:", query)
# print(comm(query,message))