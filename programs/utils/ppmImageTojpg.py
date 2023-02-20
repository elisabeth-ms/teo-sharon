import os
from PIL import Image

path = os.getcwd()
print ("The current working directory is %s" % path)
pathJpg = path + '/jpg/'
try:
    os.mkdir(pathJpg)
except OSError:
    print ("Creation of the directory %s failed" % pathJpg)
else:
    print ("Successfully created the directory %s " % pathJpg)
    
filesStr = [f for f in os.listdir(path) if os.path.isfile(os.path.join(path, f))]

filesStrOrdered = ["" for i in range(len(filesStr))]
filesStr.sort(reverse=False)
print(filesStr)
    
for image in filesStr:
    print("Ppm image: ",image,"to jpg")
    jpgImageFile = pathJpg + image[:-4]
    
    im = Image.open(path +"/"+ image)
    im.save(jpgImageFile+".jpg")