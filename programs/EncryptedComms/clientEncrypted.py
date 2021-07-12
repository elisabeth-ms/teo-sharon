# from Crypto.PublicKey import RSA
# from Crypto.Random import get_random_bytes
# from Crypto.Cipher import AES, PKCS1_OAEP

# data = "I met aliens in UFO. Here is the map.".encode("utf-8")
# file_out = open("encrypted_data.bin", "wb")

# recipient_key = RSA.import_key(open("receiver.pem").read())
# session_key = get_random_bytes(16)

# # Encrypt the session key with the public RSA key
# cipher_rsa = PKCS1_OAEP.new(recipient_key)
# enc_session_key = cipher_rsa.encrypt(session_key)

# # Encrypt the data with the AES session key
# cipher_aes = AES.new(session_key, AES.MODE_EAX)
# ciphertext, tag = cipher_aes.encrypt_and_digest(data)
# for x in (enc_session_key, cipher_aes.nonce, tag, ciphertext):
#     print(x)
#     file_out.write(x)

# file_out.close()

# from Crypto.PublicKey import RSA
# from Crypto.Cipher import AES, PKCS1_OAEP

# file_in = open("encrypted_data.bin", "rb")

# private_key = RSA.import_key(open("private.pem").read())

# enc_session_key, nonce, tag, ciphertext = \
#    [ file_in.read(x) for x in (private_key.size_in_bytes(), 16, 16, -1) ]

# # Decrypt the session key with the private RSA key
# cipher_rsa = PKCS1_OAEP.new(private_key)
# session_key = cipher_rsa.decrypt(enc_session_key)

# # Decrypt the data with the AES session key
# cipher_aes = AES.new(session_key, AES.MODE_EAX, nonce)
# data = cipher_aes.decrypt_and_verify(ciphertext, tag)
# print(data.decode("utf-8"))



import sys
import pickle
import os
from socket import *
from struct import pack
import time
from Crypto.Cipher import AES
from Crypto.Random import get_random_bytes
key = b'\x0bGG\xdb\x81\t\xe43\xfau\x93\x94\x03@\xc53\xa2\xa1\x88X\xa2\x95\xc0\x9a\xf1\x04\xdf\x9e\xefvo\x81'
print(key)
### ----------------- Just for the example ----------------------###
cwd = os.getcwd()
pathImages = cwd + '/cup__2021-01-29_04-57-47-e2cc26b5'
fixationsFileStr = pathImages+'/fixations.pkl'

def get_frames_fixations_from_pkl(fixationsFileStr):
    with open(fixationsFileStr, 'rb') as f:
        data = pickle.load(f)
    frames = data['frames']
    fixations = data['fixations']
    return frames, fixations

frames, fixations = get_frames_fixations_from_pkl(fixationsFileStr)
frame_number = 0
### ----------------- Just for the example ----------------------##



class ClientProtocol:

    def __init__(self):
        self.socket = None

    def connect(self, server_ip, server_port):
        self.socket = socket(AF_INET, SOCK_STREAM)
        self.socket.connect((server_ip, server_port))

    def close(self):
        self.socket.shutdown(SHUT_WR)
        self.socket.close()
        self.socket = None

    def send_image(self, image_data):

        # use struct to make sure we have a consistent endianness on the length
        length = pack('>Q', len(image_data))
        
        cipher_encrypt = AES.new(key, AES.MODE_CFB, iv = 16 * b'\x00')
        ciphered_bytes = cipher_encrypt.encrypt(length)
        print(ciphered_bytes)
        # sendall to make sure it blocks if there's back-pressure on the socket
        self.socket.sendall(ciphered_bytes)
        # ciphered_bytes = cipher_encrypt.encrypt(image_data)
        # self.socket.sendall(ciphered_bytes)

        ack = self.socket.recv(1)
        print(ack)

        # could handle a bad ack here, but we'll assume it's fine.

if __name__ == '__main__':
    cp = ClientProtocol()

    image_data = None

    cp.connect('127.0.0.1', 55555)
    while frame_number<1:
        with open(pathImages+"/"+frames[frame_number], 'rb') as fp:
            image_data = fp.read()
            print(len(image_data))
            # print(image_data)
            print("Lets send the new image")
            cp.send_image(image_data)
            print("Image sent")
        frame_number=frame_number+1
        time.sleep(0.2)
    cp.close()
    sys.exit()
