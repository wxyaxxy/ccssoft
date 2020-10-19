import os
import sys

def find_process(command):
    global zi
    mudiLine = None
    os.system("ps -ax |grep '"+command+"' >buffer")
    with open('buffer','r') as f:
        lines = f.readlines()
        for line in lines:
            if line.find(command)!=-1 and line.find("sudo")==-1:
                mudiLine=line
                break
    os.system("rm buffer")
    if mudiLine is not None:
        fenkai = mudiLine.split(' ')
        for zi in fenkai:
            if zi.isdigit() is True:
                break
        return zi
    else:
        return 0

# if len(sys.argv)>1:
#     mingling = sys.argv[1]

firmware_file="FW_AoShi_20200920.py"
mingling="python3 runtime/"+firmware_file
pid = find_process(mingling)
if pid != 0:
    print("find that process")
    print("pid is "+pid)
    os.system("sudo kill -9 "+pid)
    print("kill complete")
    #live
    os.chdir("/home/pi/ccssoft")
    os.system("sudo cp firmware/"+firmware_file+" runtime/"+firmware_file)
    os.system("sudo python3 runtime/"+firmware_file+"&")
else:
    print("no that process")

