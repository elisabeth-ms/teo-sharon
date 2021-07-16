import os
from socket import *
from struct import unpack
from PIL import Image
import io
from Crypto.Cipher import AES
from Crypto.Random import get_random_bytes
import base64
import numpy as np


class ServerProtocol:

    def __init__(self):
        self.socket = None
        self.output_dir = '.'
        self.file_num = 1
        self.connection = None
        self.cipher_decrypt = None
        self.cipher_encrypt = None

    
    def create_cipher_encrypt_decrypt(self):
        with open("keyEncryption.bin", mode='rb') as file:  
            key = file.read()
        with open("ivEncryption.bin", mode='rb') as file:
            iv = file.read()   
        
            
        self.cipher_encrypt = AES.new(key, AES.MODE_CFB, iv=iv)
        self.cipher_decrypt = AES.new(key, AES.MODE_CFB, iv=iv)



    def listen(self, server_ip, server_port):
        self.socket = socket(AF_INET, SOCK_STREAM)
        self.socket.bind((server_ip, server_port))
        self.socket.listen(1)
    
    def receive_and_decrypt_length_data(self, n_bytes):
        bs = self.connection.recv(n_bytes)
        if(len(bs) == n_bytes):
            deciphered_bytes = self.cipher_decrypt.decrypt(bs)
            (length,) = unpack('>Q', deciphered_bytes)
            return True, length
        else:
            return False, 0
    
    def receive_and_decrypt_image_data(self, length_bytes):
        data = b''
        data_deciphered = b''

        while len(data_deciphered) < length_bytes:
            # receiving the image in batches
            to_read = length_bytes - len(data_deciphered)
            data = self.connection.recv(4096 if to_read > 4096 else to_read)
            data_deciphered += self.cipher_decrypt.decrypt(data)
        if(data_deciphered):
            return True, data_deciphered
        else:
            return False, data_deciphered
    
    def receive_and_decrypt_array(self, length_bytes):
        data_deciphered = b''
        data = self.connection.recv(length_bytes)
        data_deciphered = self.cipher_decrypt.decrypt(data)
        
        data_decoded = base64.decodebytes(data_deciphered)                    
        data_array = np.frombuffer(data_decoded, dtype=np.float64)
        
        if data_array.size == 0:
            return False, np.array([])
        else:
            return True, data_array 


    def handle_images(self):
        print("Lets accept the connection")
        (self.connection, addr) = self.socket.accept()
        print("Connection accept")
        ok = False
        while self.connection:
            try:
                ok, length_image_data = self.receive_and_decrypt_length_data(8) # Receive and decipher the number of bytes that the image data occupy
                
                ok, length_fixation_data = self.receive_and_decrypt_length_data(8) # Receive and decipher the number of bytes that the fixation data occupy
                                
                ok, length_probability_vector = self.receive_and_decrypt_length_data(8) # Receive and decipher the number of bytes that the probability vector data occupy
                
                ok, image_data = self.receive_and_decrypt_image_data(length_image_data) # Receive and decipher the received image data
                if ok:
                    print("Image data received")
                    image = Image.open(io.BytesIO(image_data))
                    image.save("ImagesReceivedSocket/"+str(self.file_num)+".png", "PNG")
                    self.file_num += 1
             
                ok, fixation_data = self.receive_and_decrypt_array(length_fixation_data) # Receive and decipher the received fixation data 
                if ok:
                    print("Fixation point received")
                    print(fixation_data)
                
                ok, probability_vector = self.receive_and_decrypt_array(length_probability_vector) # Receive and decipher the received probability vector
                if ok:            
                    print("Probability vector received")
                    print(probability_vector)
                                
                if ok: # Send back ok
                    data_ok = b'\01'
                    print("Send Ok")                             
                    data_ok_cipher = self.cipher_encrypt.encrypt(data_ok)
                    self.connection.sendall(data_ok_cipher)
                                
            except KeyboardInterrupt:
                break


    def close(self):
        self.socket.close()
        self.socket = None
    
    def close_connections(self):
        self.connection.shutdown(SHUT_WR)
        self.connection.close()
        self.close()


if __name__ == '__main__':
    cwd = os.getcwd()
    dir = cwd +'/ImagesReceivedSocket'
    if not os.path.exists(dir):
        os.mkdir(dir)
    
    sp = ServerProtocol()
    sp.create_cipher_encrypt_decrypt()
    sp.listen('127.0.0.1', 55555)
    sp.handle_images()
    sp.close_connections()

