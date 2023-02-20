import os
from os import listdir
from os.path import isfile, join

data_directory = "/home/elisabeth/data/teoDataset11Objects/labels/"
new_directory = "/home/elisabeth/data/teoDataset11Objects/new_labels/"
if __name__ == "__main__":
    for f in listdir(data_directory):
        print(f)
        with open(data_directory+f, "r+")  as fp:
            cnt = 0
            label = 0
            x = 0
            y = 0
            w = 0
            h = 0
            for r in fp.read().split(' '):
                if cnt == 0:
                    label = int(r)
                if cnt ==1:
                    x = float(r)
                if cnt == 2:
                    y = float(r)
                if cnt ==3:
                    w = float(r)
                if cnt == 4:
                    h = float(r)
                cnt += 1
            print(label, x, y, w, h)
            if label <= 4:
                new_label = label
            elif label == 8:
                new_label = 5
            elif label == 14:
                new_label = 6
            elif label == 15:
                new_label = 7
            elif label == 16:
                new_label = 8
            elif label == 17:
                new_label = 9
            elif label == 20:
                new_label = 10 
        new_labels_file  = open(new_directory+f, "w+")
        new_labels_file.write(str(new_label)+" "+str(x)+" "+str(y)+" " + str(w)+" "+str(h)+"\n")
        new_labels_file.close()
