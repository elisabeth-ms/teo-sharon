#!/usr/bin/python3

import sys
from os import listdir, rename, mkdir, getcwd
from os.path import isfile, join
from shutil import copyfile

if __name__ == "__main__":
    try:
        filesDirectory = sys.argv[1]
        start = int(sys.argv[2])
        ext = sys.argv[3]

    except:
        print('Please pass the starting number')
    

    print('Files directory: ', filesDirectory)
    print('Starting number for renaming: ', start)
    print('Selected extension of files: ', ext)

    if ext!= '.ppm' and ext!='.float' and ext!='.txt' and ext!='.jpg':
        sys.exit(0)
    
    path = getcwd()
    print ("The current working directory is %s" % path)
    path = path + '/rename'
    try:
        mkdir(path)
    except OSError:
        print ("Creation of the directory %s failed" % path)
    else:
        print ("Successfully created the directory %s " % path)
    
    filesStr = [f for f in listdir(filesDirectory) if isfile(join(filesDirectory, f))]

    filesStrOrdered = ["" for i in range(len(filesStr))]
    filesStr.sort(reverse=False)
    print(filesStr)

    num = start
    for i in range(len(filesStr)):
        currentFile = filesStr[i]
        print(currentFile)
        if num<10:
            renamedFile = path + '/' + '0000000'+str(num)+ext
            print(renamedFile)
            copyfile(filesDirectory + currentFile, renamedFile)
            print(currentFile, ' renamed to ', renamedFile)
        elif num>=10 and num<100:
            renamedFile = path + '/' + '000000'+str(num)+ext
            copyfile(filesDirectory + currentFile, renamedFile)
            print(currentFile, ' renamed to ', renamedFile)
        elif num>=100 and num<1000:
            renamedFile = path + '/' + '00000'+str(num)+ext
            copyfile(filesDirectory + currentFile, renamedFile)
            print(currentFile, ' renamed to ', renamedFile)
        elif num>=1000 and num<10000:
            renamedFile = path + '/' + '0000'+str(num)+ext
            copyfile(filesDirectory + currentFile, renamedFile)
            print(currentFile, ' renamed to ', renamedFile)
        num += 1

    print('Files renamed in path: ', path)
    filesStr = [f for f in listdir(path) if isfile(join(path, f))]
    filesStr.sort(reverse=False)
    print(filesStr)

