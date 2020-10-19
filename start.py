import os
import json


if __name__ == "__main__":
    os.chdir('/home/pi/ccssoft')
    if os.path.exists('runtime') is not True:
        os.mkdir('runtime')
    with open('initial.ini','r') as f:
        file_json = f.read()
        file_dict = json.loads(file_json)
        runfile = file_dict['runfile']
        print(runfile)
        os.system('cp firmware/'+runfile+' runtime/')
        os.system('sudo python3 runtime/'+runfile)
