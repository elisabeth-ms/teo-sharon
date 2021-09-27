import os
from socket import *
from struct import unpack
from PIL import Image
import io
from Crypto.Cipher import AES
from Crypto.Random import get_random_bytes
import base64
import numpy as np
import yarp
import sys

class ServerProtocol(yarp.RFModule):

    def __init__(self):
        yarp.RFModule.__init__(self)

        self.socket = None
        self.output_dir = '.'
        self.file_num = 1
        self.connection = None
        self.cipher_decrypt = None
        self.cipher_encrypt = None
        self.imageWidth = 0
        self.imageHeight = 0
        self.glasses_images_port = yarp.BufferedPortImageRgb() # Create a port
        self.glasses_data_port = yarp.Port()
        self.wait_for_connection = True
        self.server_ip = '127.0.0.1'#'163.117.150.88' #'2.2.2.109'
        
    def configure(self, rf):
        self.imageWidth = rf.find("width").asInt32()
        self.imageHeight = rf.find("height").asInt32()
        
        # Open the yarp ports in the yarp network
        self.glasses_images_port.open("/glassesServer/images:o") # Give name to the port in the yarp network
        self.glasses_data_port.open("/glassesServer/data:o") 
        self.create_cipher_encrypt_decrypt()
            
        self.listen(self.server_ip, 55555)
        # sp.handle_images()

        print("RFModule configure. Running the model")
        return True
        
    def interruptModel(self):
        print("Stopping the module")
        self.glasses_images_port.interrupt()
        self.glasses_data_port.interrupt()
        return True
    
    def close(self):
        self.glasses_images_port.close()
        self.glasses_data_port.close()
        self.close_connections()

        return True
    
    def getPeriod(self):
        """
        Module refresh rate
        
        Returns: The period of the module in seconds.
        """
        return 0.1
    
    def create_cipher_encrypt_decrypt(self):
        with open(os.path.join(sys.path[0],"keyEncryption.bin"), mode='rb') as file:  
            key = file.read()
        with open(os.path.join(sys.path[0],"ivEncryption.bin"), mode='rb') as file:
            iv = file.read()
        
            
        self.cipher_encrypt = AES.new(key, AES.MODE_CFB, iv=iv)
        self.cipher_decrypt = AES.new(key, AES.MODE_CFB, iv=iv)



    def listen(self, server_ip, server_port):
        self.socket = socket(AF_INET, SOCK_STREAM)
        self.socket.setsockopt(SOL_SOCKET, SO_KEEPALIVE, 1)
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



    def updateModule(self):
        ''' 
        This function is called periodically every getPeriod() seconds
        '''
        print("Lets accept the connection")
        

        print("Connection accept")
        ok = False
        
        while True:
            if self.wait_for_connection:
                (self.connection, addr) = self.socket.accept()
                self.wait_for_connection = False
            else:
                try:
                    ok, length_image_data = self.receive_and_decrypt_length_data(8) # Receive and decipher the number of bytes that the image data occupy
                
                    ok, image_data = self.receive_and_decrypt_image_data(length_image_data) # Receive and decipher the received image data

     
                    if ok:
                        print("Image data received")
                        image = Image.open(io.BytesIO(image_data))
                        #image.save("ImagesReceivedSocket/"+str(self.file_num)+".png", "PNG")
                        #image = Image.open("ImagesReceivedSocket/"+str(self.file_num)+".png")
                        width, height = image.size
                        
                        #yarpview has problems with the visualization of images of some resolutions. Since the images received are almost squared, we are 
                        # going to crop the image
                        square_length = 0
                        if width<height:
                            square_length = width
                        else:
                            square_length = height
                        
                        image = image.crop((0,0,square_length,square_length))

                        img_array = np.asarray(image)
                        #print(img_array)
                        #self.file_num += 1

                        yarp_img = self.glasses_images_port.prepare() # Get a place to store the image to be sent
            
                        img = yarp.ImageRgb()
                
                        img.resize(square_length, square_length)
                        img.setExternal(img_array.data, square_length, square_length) # Wrap yarp image around the numpy array  
                        yarp_img.copy(img)
                        self.glasses_images_port.write()                    
                    
                    ok, length_fixation_data = self.receive_and_decrypt_length_data(8) # Receive and decipher the number of bytes that the fixation data occupy
    
                    ok, fixation_data = self.receive_and_decrypt_array(length_fixation_data) # Receive and decipher the received fixation data 
                    if ok:
                        print("Fixation point received")
                        print(fixation_data)
                    
                    ok, length_probability_vector = self.receive_and_decrypt_length_data(8) # Receive and decipher the number of bytes that the probability vector data occupy

                    ok, probability_vector = self.receive_and_decrypt_array(length_probability_vector) # Receive and decipher the received probability vector
                    if ok:            
                        print("Probability vector received")
                        print(probability_vector)
                    
                        bdata_fixation = yarp.Bottle()
                        bdata_fixation.addString("fixation_point")
                        bpoint = bdata_fixation.addList()
                        bpoint.addDouble(fixation_data[0])
                        bpoint.addDouble(fixation_data[1])
                        bdata_fixation.addString("probability_vector")
                        bprob = bdata_fixation.addList()
                        for prob in probability_vector:
                            bprob.addDouble(prob)
                    
                        self.glasses_data_port.write(bdata_fixation)

                    if ok: # Send back ok
                        data_ok = b'\01'
                    else:
                        data_ok = b'\00'
                                                 
                    data_ok_cipher = self.cipher_encrypt.encrypt(data_ok)
                    try:
                        self.connection.sendall(data_ok_cipher)
                    except:
                        self.close_connections()
                        self.create_cipher_encrypt_decrypt()
                        self.listen(self.server_ip, 55555)
                        self.wait_for_connection = True
                                     
                except KeyboardInterrupt:
                    break
        return True


    # def close(self):
    #     self.socket.close()
    #     self.socket = None
    
    def close_connections(self):
        print("Closing the connections...")
        try:
            self.connection.shutdown(SHUT_RDWR)
        except:
            pass
        self.connection.close()
        self.socket.close()
        self.socket = None


if __name__ == '__main__':
    cwd = os.getcwd()
    dir = cwd +'/ImagesReceivedSocket'
    if not os.path.exists(dir):
        os.mkdir(dir)
    
    yarp.Network.init()  # connect to YARP network

    if not yarp.Network.checkNetwork():  # let's see if there was actually a reachable YARP network
        print('yarp network is not found!!')   
        sys.exit(1)
    
    sp = ServerProtocol()
    
    rf = yarp.ResourceFinder()
    rf.setVerbose(True)
    rf.setDefaultContext('serverEncrypted')
    rf.setDefaultConfigFile('serverEncrypted.ini')
    rf.configure(sys.argv)
    
    sp.runModule(rf)

    sys.exit()


