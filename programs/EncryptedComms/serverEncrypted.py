import os
from socket import *
from struct import unpack
from PIL import Image
import io
from Crypto.Cipher import AES
from Crypto.Random import get_random_bytes

image_data = ... # byte values of the image
key = b'\x0bGG\xdb\x81\t\xe43\xfau\x93\x94\x03@\xc53\xa2\xa1\x88X\xa2\x95\xc0\x9a\xf1\x04\xdf\x9e\xefvo\x81'


class ServerProtocol:

    def __init__(self):
        self.socket = None
        self.output_dir = '.'
        self.file_num = 1
        self.connection = None

    def listen(self, server_ip, server_port):
        self.socket = socket(AF_INET, SOCK_STREAM)
        self.socket.bind((server_ip, server_port))
        self.socket.listen(1)

    def handle_images(self):
        try:
            while True:
                print("Lets accept the connection")
                (self.connection, addr) = self.socket.accept()
                print("Connection accept")
                while True:
                    try:
                        bs = self.connection.recv(8)
                       
                        if(len(bs)==8):
                            print(bs)
                            cipher_decrypt = AES.new(key, AES.MODE_CFB,  iv = 16 * b'\x00')
                            print("lets decrypt")

                            deciphered_bytes = cipher_decrypt.decrypt(bs)
                            print("decrypted")
                            print("Deciphered bytes")
                            print(deciphered_bytes)
                            (length,) = unpack('>Q', deciphered_bytes)
                            print(length)
                            data = b''
                            while len(data) < length:
                                # doing it in batches is generally better than trying
                                # to do it all in one go, so I believe.
                                to_read = length - len(data)
                                data += self.connection.recv(
                                    4096 if to_read > 4096 else to_read)
                            print(len(data))
                            # send our 0 ack
                            assert len(b'\00') == 1
                            self.connection.sendall(b'\00')
                            image = Image.open(io.BytesIO(data))
                            print(image.size)
                            image.save("ImagesReceivedSocket/"+str(self.file_num)+".png", "PNG")
                            self.file_num += 1
                    except:
                        pass
        except:
            pass


    def close(self):
        self.socket.close()
        self.socket = None
    
    def close_connections(self):
        self.connection.shutdown(SHUT_WR)
        self.connection.close()
        self.close()

        # could handle a bad ack here, but we'll assume it's fine.

if __name__ == '__main__':
    sp = ServerProtocol()
    sp.listen('127.0.0.1', 55555)
    sp.handle_images()
    sp.close_connections()

