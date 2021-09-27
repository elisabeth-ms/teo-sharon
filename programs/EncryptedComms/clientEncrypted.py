import sys
import pickle
import os
from socket import *
from struct import pack
import time
from Crypto.Cipher import AES
from Crypto.Random import get_random_bytes
import numpy as np
import base64
from numpy.random import seed
from numpy.random import rand

# newKey = get_random_bytes(16)
# newFile = open("keyEncryptionOther.bin", "wb")
# newFile.write(newKey)
# newFile.close()


### ----------------- Just for the example ----------------------###


def get_frames_fixations_from_pkl(fixationsFileStr):
    with open(fixationsFileStr, 'rb') as f:
        data = pickle.load(f)
    frames = data['frames']
    fixations = data['fixations']
    action_level = data['action_level']
    print(action_level)
    return frames, fixations, action_level


### ----------------- Just for the example ----------------------##



class ClientProtocol:

    def __init__(self):
        self.socket = None
        self.cipher_encrypt = None
        self.cipher_decrypt = None

    def create_cipher_encrypt_decrypt(self):
        with open(os.path.join(sys.path[0],"keyEncryption.bin"), mode='rb') as file:  
            key = file.read()
        with open(os.path.join(sys.path[0],"ivEncryption.bin"), mode='rb') as file:
            iv = file.read()
            
        self.cipher_encrypt = AES.new(key, AES.MODE_CFB, iv=iv)
        self.cipher_decrypt = AES.new(key, AES.MODE_CFB, iv=iv)

    def connect(self, server_ip, server_port):
        self.socket = socket(AF_INET, SOCK_STREAM)
        self.socket.connect((server_ip, server_port))

    def close(self):
        self.socket.shutdown(SHUT_WR)
        self.socket.close()
        self.socket = None


    def send_data(self, image_data, fixation, probability_vector):

        # use struct to make sure we have a consistent endianness on the length
        # Send the number of bytes in the image_data
        length = pack('>Q', len(image_data))
        ciphered_bytes = self.cipher_encrypt.encrypt(length)
        self.socket.sendall(ciphered_bytes)
        
         # Send the image data
        ciphered_bytes = self.cipher_encrypt.encrypt(image_data)
        self.socket.sendall(ciphered_bytes)
        
        # Send the number of bytes in the fixation data                 
        fixation_data_encoded = base64.b64encode(fixation)
        length_fixation_data = pack('>Q', len(fixation_data_encoded))
        ciphered_bytes = self.cipher_encrypt.encrypt(length_fixation_data)
        self.socket.sendall(ciphered_bytes)
        
        # Send the fixation data
        ciphered_bytes = self.cipher_encrypt.encrypt(fixation_data_encoded)
        self.socket.sendall(ciphered_bytes)
        
        # Send the number of bytes in the probability vector
        probability_vector_data_encoded = base64.b64encode(probability_vector)
        length_probability_vector_data = pack('>Q', len(probability_vector_data_encoded))
        ciphered_bytes = self.cipher_encrypt.encrypt(length_probability_vector_data)
        self.socket.sendall(ciphered_bytes)
        
        # Send the probability vector
        ciphered_bytes = self.cipher_encrypt.encrypt(probability_vector_data_encoded)
        self.socket.sendall(ciphered_bytes)
        
        
        ok_ciphered = self.socket.recv(1)
        ok = self.cipher_decrypt.decrypt(ok_ciphered)
        print(ok)
        

if __name__ == '__main__':
    
    if len(sys.argv) == 3:
        print(sys.argv[1])
        path_images = sys.argv[1]
        number_frames = sys.argv[2]
        fixations_file_str = path_images+'/fixations.pkl'
        

        frames, fixations, action_level = get_frames_fixations_from_pkl(fixations_file_str)
        for i in range(len(frames)):
            print(frames[i], action_level[i])
        categories = np.load(sys.path[0]+'/../demoSharon/'+'/categories.npy')

        #Lets check the category of the graspped object in the video
        selected_index_category = None
        selected_category = None
        for index_category, category in enumerate(categories):
            if category in path_images:
                selected_category = category
                selected_index_category = index_category
                break
        print('Selected category:', selected_category, selected_index_category)

        
        frame_number = 0
    
        cp = ClientProtocol()
        cp.create_cipher_encrypt_decrypt()
        image_data = None
        remote_ip =  '127.0.0.1'#'163.117.150.88' 

        cp.connect(remote_ip, 55555)
        
        while frame_number<int(number_frames):
            try:
                with open(path_images+"/"+frames[frame_number], 'rb') as fp:
                #with open('1.png', 'rb') as fp:
                    image_data = fp.read()
                    print("Lets send the new image")
                    #print(rand(categories.size))
                    
                    probability_vector = np.random.uniform(low=0.0, high=0.4, size=(categories.size,))
                    aux_prob = np.random.uniform(low=0.8, high=1.0, size=(1,))
                    
                    if action_level[frame_number] == 0:
                        probability_vector[0] = aux_prob[0]
                    else:
                        print(frames[frame_number])
                        probability_vector[selected_index_category] = aux_prob[0]
                    #print(probability_vector)
                    
                    cp.send_data(image_data, fixations[frame_number], probability_vector)
                    print("Image, fixation point and probability vector sent")
                frame_number=frame_number+1
                time.sleep(0.01)
            except KeyboardInterrupt:
                break
        cp.close()
        sys.exit()
