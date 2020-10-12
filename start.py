import os
import json
import time
import threading

def dingshi_paiZhao():
    cnt = 0
    while True:
        print("hell")
        cnt = cnt+1
        if cnt>5:
            break
        time.sleep(1)

if __name__ == "__main__":
    t3 = threading.Thread(target=dingshi_paiZhao)
    t3.start()
    t3.join()
    if t3.is_alive() is not True:
        print("here")
        t3 = threading.Thread(target=dingshi_paiZhao)
        t3.start()
    # t3.start()
    # t3.start()
    # os.system("sudo quectel-pppd.sh")
    # time.sleep(2)
    # os.system("sudo route del default&&sudo route add default dev ppp0")
    # time.sleep(1)
    # os.system("sudo phddns restart")
    # time.sleep(2)
    # os.chdir('/home/pi/ccssoft')
    # if os.path.exists('runtime') is not True:
    #     os.mkdir('runtime')
    # with open('initial.ini','r') as f:
    #     file_json = f.read()
    #     file_dict = json.loads(file_json)
    #     runfile = file_dict['runfile']
    #     print(runfile)
    #     os.system('cp firmware/'+runfile+' runtime/')
    #     os.system('sudo python3 runtime/'+runfile)
