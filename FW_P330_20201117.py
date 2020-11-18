# 程序功能

# 飞行日志记录功能
# 在飞机处于巡航状态的时候
# 创建文件，文件名为missionId.track
# 每秒将当前的lon,lat,alt写入到文件中，信息格式为“lon,lat,alt\n"
# 当飞机降落到地面，停止记录，关闭文件
# 等服务器发送文件请求，将该文件发送给服务器，通过Post方式，另外一个微服务？

#修改日志
#心跳包增加时间信息

from pymavlink import mavutil
import threading
import time
from linkkit import linkkit
import json
import wget
import gphoto2
import os
import requests
import datetime
import logging
import piexif
import sys
import pymavlink.dialects.v10.common as cm

# 全局flag
flag_connect = 0
flag_ack = 0
id_command = 0
request = 0
ack = 0
read_plan_count=0
read_plan_list=[]
flag_start = 0
mission_num = "2003301112"
planFileName = 'test.plan'
url = "http://39.99.240.145:8080/filemicroservice/imageRecognition"  # 微服务的路径
photoNum=1
image_hostName = "http://47.92.112.190:8080/"  # 微服务的路径
image_luJing="filemicroservice/imageRecognition"
renzheng_name="uav"
renzheng_secret="123456"
sd_lujing = "/media/pi/SDCARD"

# 心跳json包样式
HeartBeat = {'Type': 'HeartBeat',
             'base_mode': 4,
             'custom_mode': 0,
             'mission_id': mission_num,
             'PhotoNum': 1,
             'system_status': 0,
             'battery_remaining': 90,
             'voltage_battery':0,
             'airspeed': 15,
             'groundspeed': 20,
             'image_url':image_hostName+image_luJing,
             'lat': 0,
             'lon': 0,
             'alt': 20,
             'yaw': 0,
             'roll': 0,
             'pitch': 0,
             'UAV_Firmware': 'DF_20',
             'device_id':"device1",
             'time':'2020-05-17 10:39:20',
             'sate_num':0,
             'dpgs_num': 0,
             'SD':'1'
             }


def read_init(whichserver):
    global renzheng_name
    global renzheng_secret
    global image_luJing
    global image_hostName
    global mission_num
    product_key = "a1DgsgxAjXK"  # 默认参数
    device_name = "device1"  # 每个设备写死的参数
    device_secret = "nXCVzfnyht4veJaaGKqv3j4lfzOBIMv6"  # 默认参数
    product_secret = ""
    try:
        with open('initial.ini','r') as f:
            initdata=f.read()
        logger.info("init data open success")
        init_dict = json.loads(initdata)
        try:
            product_key = init_dict[whichserver]['product_key']
        except:
            pass
        try:
            product_secret = init_dict[whichserver]['product_secret']
        except:
            pass
        try:
            device_secret = init_dict[whichserver]['device_secret']
        except:
            pass
        try:
            device_name=init_dict[whichserver]['device_name']
        except:
            pass
        try:
            renzheng_secret=init_dict['image_post']['passwd']
            renzheng_name=init_dict['image_post']['user']
            image_hostName=init_dict['image_post']['hostName']
            image_luJing=init_dict['image_post']['luJing']
            HeartBeat['image_url']=image_hostName+image_luJing
            mission_num=init_dict['mission_num']
            HeartBeat['mission_id']=mission_num
            logger.info("read image url succes")
            logger.info("image_url is " + image_hostName + image_luJing)
        except:
            logger.info("read init date error")
    except:
        logger.info("use default init data")
    return product_key,product_secret,device_secret,device_name


def duTodufenmiao(du):
    result_du = (int(du))
    result_fen = (int((du - int(du)) * 60))
    result_miao = ((((du - int(du)) * 60) - int((du - int(du)) * 60)) * 60)
    result_miao = int(result_miao * 100)  # 四舍五入 保留4位小数点
    return result_du, result_fen, result_miao


def modify(filename, latitude, longitude, altitude, yaw, roll, pitch):
    image_dict = piexif.load(filename)
    du_lat, fen_lat, miao_lat = duTodufenmiao(latitude)
    du_long, fen_long, miao_long = duTodufenmiao(longitude)
    altitude = int(altitude)
    yaw = abs(int(yaw * 100))  # 不能写入负数，当前求绝对值，应该用反码
    roll = abs(int(roll * 100))
    pitch = abs(int(pitch * 100))
    gps_ifd = {
        piexif.GPSIFD.GPSVersionID: (2, 0),
        piexif.GPSIFD.GPSLatitudeRef: 'N',
        piexif.GPSIFD.GPSLatitude: ((du_lat, 1), (fen_lat, 1), (miao_lat, 100)),
        piexif.GPSIFD.GPSLongitudeRef: 'E',
        piexif.GPSIFD.GPSLongitude: ((du_long, 1), (fen_long, 1), (miao_long, 100)),
        piexif.GPSIFD.GPSAltitudeRef: 0,
        piexif.GPSIFD.GPSAltitude: (altitude, 1),
        piexif.GPSIFD.GPSDOP: (30, 1),
        piexif.GPSIFD.GPSSpeed: (yaw, 100),
        piexif.GPSIFD.GPSTrack: (roll, 100),
        piexif.GPSIFD.GPSImgDirection: (pitch, 100)
    }
    dateStamp = datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S")
    image_dict['GPS'] = gps_ifd
    image_dict['Exif'][piexif.ExifIFD.DateTimeOriginal] = dateStamp
    image_dict['0th'][piexif.ImageIFD.Orientation]=0#让照片保持横着，不要竖着

    image_byte = piexif.dump(image_dict)
    piexif.insert(image_byte, filename)

def jiexi_P330(file):
    with open(file,'r') as f:
        a = f.read()
    a_dict = json.loads(a)
    waypoint_num = len(a_dict['mission']['items'])
    Total_waypoint = "{\"waypoint\":["
    mis_dict = {}
    for i in range(waypoint_num):
        mis_dict["param1"] = a_dict['mission']['items'][i]["params"][0]
        mis_dict["param2"] = a_dict['mission']['items'][i]["params"][1]
        mis_dict["param3"] = a_dict['mission']['items'][i]["params"][2]
        if a_dict['mission']['items'][i]["params"][3] is None:
            mis_dict["param4"] = 0
        else:
            mis_dict["param4"] = a_dict['mission']['items'][i]["params"][3]
        mis_dict["x"] = int(a_dict['mission']['items'][i]["params"][4]* 10000000)
        mis_dict["y"] = int(a_dict['mission']['items'][i]["params"][5]* 10000000)
        mis_dict["z"] = a_dict['mission']['items'][i]["params"][6]* 1.0
        mis_dict["seq"] = a_dict['mission']['items'][i]["doJumpId"]-1
        mis_dict["command"] = a_dict['mission']['items'][i]["command"]
        # mis_dict["frame"] = a_dict['mission']['items'][i]["frame"]
        if i!=(waypoint_num-1):
            mis_dict["current"] = 0
            mis_dict["frame"] = 5
        else:
            mis_dict["current"] = 1
            mis_dict["frame"] = 2
        # mis_dict["current"] = a_dict['mission']['items'][i]["doJumpId"]
        # mis_dict["autocontinue"] = a_dict['mission']['items'][i]["autoContinue"]
        mis_dict["autocontinue"] =1
        Total_waypoint = Total_waypoint +json.dumps(mis_dict)+ ','

    Total_waypoint = Total_waypoint[:-1] + "]}"
    with open('plan_write.json','w+') as f:
        f.write(Total_waypoint)
    # return Total_waypoint #is json
    # logger.info(Total_waypoint)
    return json.loads(Total_waypoint)# is string or dict

def xiaFangHangXian():
    # 清除原有的mission
    the_connect.mav.mission_clear_all_send(the_connect.target_system, the_connect.target_component)
    if wait_for_ack(0.5):
        mis_dict = jiexi_P330("planfile/"+planFileName)
        length = len(mis_dict['waypoint'])
        logger.info("waypoint num is %d" % length)
        the_connect.mav.mission_count_send(the_connect.target_system, the_connect.target_component, length)
        if wait_for_ack(0.5):
            logger.info("start to send waypoint")
            for i in range(length):
                logger.info("sending waypoint %d" % i)
                the_connect.mav.mission_item_int_send(the_connect.target_system, the_connect.target_component, i,
                                                      mis_dict['waypoint'][i]['frame'],
                                                      mis_dict['waypoint'][i]['command'],
                                                      mis_dict['waypoint'][i]['current'],
                                                      mis_dict['waypoint'][i]['autocontinue'],
                                                      mis_dict['waypoint'][i]['param1'],
                                                      mis_dict['waypoint'][i]['param2'],
                                                      mis_dict['waypoint'][i]['param3'],
                                                      mis_dict['waypoint'][i]['param4'],
                                                      mis_dict['waypoint'][i]['x'],
                                                      mis_dict['waypoint'][i]['y'],
                                                      mis_dict['waypoint'][i]['z']
                                                      )
                if i!=(length-1):
                    if wait_for_ack(0.1):
                        continue
                    else:
                        return False
                else:
                    break
        else:
            return False
    else:
        return False
    if wait_for_ack(0.1):
        return True
    else:
        return False

def plan_check():
    with open('plan_write.json','r') as f:
        write_data_json = f.read()
    with open('plan_read.json','r') as f:
        read_data_json = f.read()
    write_data_dict = json.loads(write_data_json)
    read_data_dict = json.loads(read_data_json)
    for i in range(len(write_data_dict["waypoint"])):
        if abs(write_data_dict["waypoint"][i]["x"]-read_data_dict["waypoint"][i]["x"])<2 and abs(write_data_dict["waypoint"][i]["y"]-read_data_dict["waypoint"][i]["y"])<2 and write_data_dict["waypoint"][i]["z"]==read_data_dict["waypoint"][i]["z"] and write_data_dict["waypoint"][i]["command"]==read_data_dict["waypoint"][i]["command"]:
            continue
        else:
            return False
    return True
    # if write_data_dict==read_data_dict:
    #     return True
    # else:
    #     return False
    
def read_planfile():
    global read_plan_count
    global read_plan_list
    the_connect.mav.mission_request_list_send(the_connect.target_system,the_connect.target_component)
    #logger.info(read_plan_count)
    if wait_for_ack(0.5):
        logger.info(read_plan_count)
        read_plan_list.clear()
        for read_plan_index in range(read_plan_count):
            logger.info("staring read waypoint is "+str(read_plan_index))
            the_connect.mav.mission_request_int_send(the_connect.target_system,the_connect.target_component,read_plan_index)
            if wait_for_ack(0.1):
                continue
            else:
                return False
        read_plan_dict={"waypoint":read_plan_list}
        with open('plan_read.json','w+') as f:
            f.write(json.dumps(read_plan_dict))
        # logger.info(json.dumps(read_plan_dict))
        return True
    else:
        return False
    

def read_mavlink():  # 读mavlink数据放入心跳json包
    global ack
    global read_plan_count
    global read_plan_list
    global flag_start
    while True:
        the_connect.recv_match(blocking=True)
        #logger.info(the_connect.messages)
        #logger.info(the_connect.mavlink10())
        try:
            HeartBeat['base_mode'] = the_connect.messages['HEARTBEAT'].base_mode
            HeartBeat['custom_mode'] = the_connect.messages['HEARTBEAT'].custom_mode
            # HeartBeat['custom_mode'] = 67371008 #debug
            main_mode = (HeartBeat['custom_mode']>>16)&0xff
            sub_mode = (HeartBeat['custom_mode']>>24)&0xff
            HeartBeat['system_status'] = the_connect.messages['HEARTBEAT'].system_status
            if HeartBeat['system_status']==3:
                HeartBeat['base_mode']=4#准备就绪
            elif HeartBeat['system_status']==4:
                HeartBeat['base_mode']=5#解锁完成
            elif HeartBeat['system_status']==7:
                HeartBeat['base_mode']=10#关机
            elif HeartBeat['system_status']==5:
                HeartBeat['base_mode']=11#有异常
            if main_mode==4:
                if sub_mode==4:
                    HeartBeat['base_mode']=7#巡航中
                    flag_start=1
                elif sub_mode==5:
                    HeartBeat['base_mode']=13#返航中
                    flag_start=0
                elif sub_mode==6:
                    HeartBeat['base_mode']=8#降落中
        except:
            pass
        try:
            HeartBeat['battery_remaining'] = the_connect.messages['SYS_STATUS'].battery_remaining
            HeartBeat['voltage_battery'] = the_connect.messages['SYS_STATUS'].voltage_battery
            if HeartBeat['battery_remaining'] >100:
                HeartBeat['battery_remaining']=100
            pass
        except:
            pass
        try:
            HeartBeat['airspeed'] = round(the_connect.messages['VFR_HUD'].airspeed, 2)
            HeartBeat['groundspeed'] = round(the_connect.messages['VFR_HUD'].groundspeed, 2)
            pass
        except:
            pass
        try:
            HeartBeat['roll'] = int(round(the_connect.messages['ATTITUDE'].roll, 2)*57.3)
            yaw = int(round(the_connect.messages['ATTITUDE'].yaw, 2)*57.3)
            if yaw<0:
                HeartBeat['yaw'] = 360+yaw 
            else:
                HeartBeat['yaw']=yaw
            HeartBeat['pitch'] = int(round(the_connect.messages['ATTITUDE'].pitch, 2)*57.3)
        except:
            pass
        try:
            HeartBeat['lon'] = round(the_connect.messages['GLOBAL_POSITION_INT'].lon/10000000,6)
            HeartBeat['lat'] = round(the_connect.messages['GLOBAL_POSITION_INT'].lat/10000000,6)
            HeartBeat['alt'] = int(the_connect.messages['GLOBAL_POSITION_INT'].alt/1000)
        except:
            pass
        try:
            command = the_connect.messages['COMMAND_ACK'].command
            result = the_connect.messages['COMMAND_ACK'].result
            logger.info("command is %d result is %d" % (command, result))
            ack=result
        except:
            pass
        try:
            read_plan_count = the_connect.messages['MISSION_COUNT'].count
            ack=0
        except:
            pass
        try:
            read_dict={}
            read_dict['param1'] = the_connect.messages['MISSION_ITEM_INT'].param1
            read_dict['param2'] = the_connect.messages['MISSION_ITEM_INT'].param2
            read_dict['param3'] = the_connect.messages['MISSION_ITEM_INT'].param3
            read_dict['param4'] = the_connect.messages['MISSION_ITEM_INT'].param4
            read_dict['x'] = the_connect.messages['MISSION_ITEM_INT'].x
            read_dict['y'] = the_connect.messages['MISSION_ITEM_INT'].y
            read_dict['z'] = the_connect.messages['MISSION_ITEM_INT'].z
            read_dict['seq'] = the_connect.messages['MISSION_ITEM_INT'].seq
            read_dict['command'] = the_connect.messages['MISSION_ITEM_INT'].command
            read_dict['current'] = the_connect.messages['MISSION_ITEM_INT'].current
            read_dict['frame'] = the_connect.messages['MISSION_ITEM_INT'].frame
            read_dict['autocontinue'] = the_connect.messages['MISSION_ITEM_INT'].autocontinue
            read_plan_list.append(read_dict)
            ack=0
            del the_connect.messages['MISSION_ITEM_INT']
        except:
            pass
        try:
            seq = the_connect.messages['MISSION_REQUEST_INT'].seq
            ack = 0
        except:
            pass
        try:
            seq = the_connect.messages['MISSION_REQUEST'].seq
            ack = 0
        except:
            pass
        try:
            seq = the_connect.messages['MISSION_ACK'].type
            ack = 0
        except:
            pass
        # try:
        #     sate_num=the_connect.messages['GPS2_RAW'].satellites_visible
        #     dgps_numch=the_connect.messages['GPS2_RAW'].dgps_numch
        #     logger.info("sate_num is "+str(sate_num))
        #     logger.info("dgps_numch is "+str(dgps_numch))
        # except:
        #     pass
        the_connect.messages.clear()

def find_process(command):
    global zi
    mudiLine=None
    os.system("ps -ax>buffer")
    with open('buffer','r') as f:
        lines=f.readlines()
        for line in lines:
            if line.find(command)!=-1:
                mudiLine=line
                break
    #os.system("rm buffer")
    if mudiLine is not None:
        fenkai = mudiLine.split(' ')
        for zi in fenkai:
            if zi.isdigit() is True:
                break
        return zi
    else:
        return 0
def find_ppp0():
    os.system("ifconfig|grep ppp0>findppp0")
    with open('findppp0','r') as f:
        lines=f.readlines()
        for line in lines:
            if line.find('ppp0')!=-1:
                logger.info("find ppp0")
                return True
    logger.info("not find ppp0")
    return False

flag_mount=0
flagHeartBeatSendSuccess=1
heartBeatList=[]

def resetNet():
    global flagHeartBeatSendSuccess
    if find_process("pppd")==0:
        logger.info("net error...start reset net")
        os.system("sudo route del default")
        time.sleep(1)
        err_count=0
        while find_process("pppd")==0:
            # if os.path.exists("/dev/ttyU")
            os.system("sudo quectel-pppd.sh")
            time.sleep(5)
            logger.info("reset net....")
        while find_ppp0() is False:
            logger.info("waiting for ppp0 ...")
            err_count=err_count+1
            if err_count>2:
                return 0
            time.sleep(1)
        os.system("sudo route add default dev ppp0")
        # os.execl("/usr/bin/python3", "python3", "start.py")
        lk.disconnect()
        lk.connect_async()  # 启动连接

        #time.sleep(1)
        #os.system("sudo phddns restart")
        #time.sleep(2)
    else:
        # logger.info("net is working")
        pass

def on_publish_topic(mid, userdata):  # 发送消息成功与否回调函数
    global flagHeartBeatSendSuccess
    flagHeartBeatSendSuccess=1
    logger.info("on_publish_topic mid:%d" % mid)

def heartBeatSend():
    global heartBeatList
    global flagHeartBeatSendSuccess
    global flag_connect
    while True:
        #总是发完，因为每秒往里面存数据
        if len(heartBeatList)>=1:
            if 1 == flag_connect and flagHeartBeatSendSuccess == 1:
                try:
                    lk.publish_topic(lk.to_full_topic("user/update"), heartBeatList[0], 1)
                    heartBeatList.pop(0)
                    flagHeartBeatSendSuccess = 0
                except Exception as e:
                    logger.info("heartBeatSend:"+str(e))
        else:
            time.sleep(1)

def heartBeatRecord():
    global flag_connect
    global HeartBeat
    global sd_lujing
    global flag_mount
    global flagHeartBeatSendSuccess
    global heartBeatList
    while True:
        HeartBeat['time'] = str(datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S"))
        heartBeatList.append(json.dumps(HeartBeat))
        if os.path.exists("/dev/sda1"):#插入了U盘
            if flag_mount==0:#未挂载
                if os.path.exists(sd_lujing) is not True:
                    os.mkdir(sd_lujing)
                os.system("sudo mount /dev/sda1 " + sd_lujing)
                logger.info("start mount SD")
                flag_mount=1
                HeartBeat['SD'] = "SD未挂载"
            else:#已挂载
                os.system("df -h|grep " + sd_lujing + ">dfbuffer")
                with open("dfbuffer", 'r') as f:
                    lines = f.read()
                mubiao = lines.split(" ")[-2]
                HeartBeat['SD'] = mubiao[:-1]
        else:#未插入
            if flag_mount==1:#之前挂载过
                os.system("sudo umount " + sd_lujing)
                flag_mount=0
                logger.info("umount sd ")
                HeartBeat['SD'] = "SD未插入"
            else:
                HeartBeat['SD'] = "SD未插入"
        time.sleep(1)

# 定时线程
def dingshi():
    while True:
        resetNet()
        time.sleep(1)


# 阿里云的各种回调函数
def on_connect(session_flag, rc, userdata):
    global flag_connect
    global flagHeartBeatSendSuccess
    logger.info("on_connect:%d,rc:%d,userdata:" % (session_flag, rc))
    # 订阅主题
    if rc==0:
        rc, mid = lk.subscribe_topic(lk.to_full_topic("user/get"))
        lk.on_subscribe_topic = on_subscribe_topic  # 订阅成功回调函数
        lk.on_topic_message = on_topic_message  # 收到消息回调函数
        flagHeartBeatSendSuccess=1
        flag_connect = 1


def on_disconnect(rc, userdata):
    logger.info("on_disconnect:rc%d,userdata:" % rc)


def on_subscribe_topic(mid, granted_qos, userdata):
    logger.info("on_subscribe_topic mid:%d,granted_qos:%s" % (mid, str(','.join('%s' % it for it in granted_qos))))
    pass

def iot_reply_error(title,e):
    info = title+" error:" + str(e)
    logger.info(info)
    msg = {'Type': title, 'Status': 'error', 'Details': str(e),'device_id': dv_name}
    if 1 == flag_connect:
        lk.publish_topic(lk.to_full_topic("user/update"), json.dumps(msg))

def iot_reply_ok(title):
    msg = {'Type': title, 'Status': 'ok', 'device_id': dv_name}
    if 1 == flag_connect:
        lk.publish_topic(lk.to_full_topic("user/update"), json.dumps(msg))

def iot_reply_ok_some(title,other):
    msg = {'Type': title, 'Status': 'ok', 'Details':other,'device_id': dv_name}
    if 1 == flag_connect:
        lk.publish_topic(lk.to_full_topic("user/update"), json.dumps(msg))


def iot_reply_other(title,other):#回复不支持当前指令等
    msg = {'Type': title, 'Status': 'error', 'Details': other,'device_id': dv_name}
    # logger.info(msg)
    if 1 == flag_connect:
        lk.publish_topic(lk.to_full_topic("user/update"), json.dumps(msg))

def iot_reply_command(command_id,result,detail):
    msg = {'Type': 'Command', 'CMD': command_id, 'Status': 'error', 'result': result, 'device_id': dv_name,'Details': detail}
    logger.info(msg)
    if result==0:
        msg['Status']="ok"
    if 1 == flag_connect:
        lk.publish_topic(lk.to_full_topic("user/update"), json.dumps(msg))

def image_url_modify(url,user,password):
    global image_hostName
    global image_luJing
    global renzheng_name
    global renzheng_secret
    server_url = url
    list_url = server_url.split('/')
    image_hostName_new = list_url[0] + '//' + list_url[2] + '/'
    image_luJing_new = list_url[3] + '/' + list_url[4]
    renzheng_name_new = user
    renzheng_secret_new = password
    if image_luJing==image_luJing_new and image_hostName_new==image_hostName and renzheng_secret==renzheng_secret_new and renzheng_name==renzheng_name_new:
        pass
    else:#如果有变动才修改记录文件
        with open('initial.ini', 'r') as f:
            write_json = f.read()
        writedata = json.loads(write_json)  # 将json转换为字典
        writedata['image_post']['hostName'] = image_hostName_new
        writedata['image_post']['luJing'] = image_luJing_new
        writedata['image_post']['user'] = renzheng_name_new
        writedata['image_post']['passwd'] = renzheng_secret_new
        write_json = json.dumps(writedata)
        with open('initial.ini', 'w') as f:
            f.write(write_json)
        logger.info("image_hostName_new is "+image_hostName_new)
        logger.info("image_luJing_new is "+image_luJing_new)
        logger.info("renzheng_name_new is "+renzheng_name_new)
        logger.info("renzheng_secret_new is "+renzheng_secret_new)
        image_hostName=image_hostName_new
        image_luJing=image_luJing_new
        renzheng_name=renzheng_name_new
        renzheng_secret=renzheng_secret_new
        HeartBeat['image_url']=image_hostName+image_luJing


def wait_for_ack(time_to_sleep):
    global ack
    ack = 10#特定值
    time_count = 0
    while ack == 10:  # 等待ack非10
        logger.info("waiting for ack")
        time_count = time_count + 1  # 计时，如果时间大于2秒，仍然没有收到应答，跳出循环
        if time_count > 10:
            break
        time.sleep(time_to_sleep)
    if time_count > 10:  # 如果时间大于5秒还未收到应答
        logger.info("timeout no disarm ack")
        #iot_reply_command(command_id, 6)#6代表超时
        return False
    elif ack==0:#成功
        #logger.info("command accept")
        #iot_reply_command(command_id,ack)
        return True
    elif ack!=0:
        logger.info("command refuse")
        #iot_reply_command(command_id,ack)
        return False
    else:
        return False

def take_a_photo(jieshou):
    global photoNum
    global mission_num
    try:
        if HeartBeat['SD'].isdigit():
            camera = gphoto2.Camera()
            camera.init()
            if mission_num!=jieshou['mission_id']:
                mission_num = str(jieshou['mission_id'])
                logger.info("mission num changed,new num is " + mission_num)
                if os.path.exists(sd_lujing + "/" + mission_num) is False:
                    os.mkdir(sd_lujing + "/" + mission_num)
                HeartBeat['mission_id'] = mission_num
                with open('initial.ini', 'r') as f:
                    write_json = f.read()
                writedata = json.loads(write_json)  # 将json转换为字典
                writedata['mission_num'] = mission_num
                write_json = json.dumps(writedata)
                with open('initial.ini', 'w') as f:
                    f.write(write_json)
                logger.info("mission_num save success")
                get_photoNum(mission_num)
            target = sd_lujing + "/" + jieshou['mission_id'] + "/" + jieshou['mission_id'] + "_" + str(photoNum) + ".jpg"
            logger.info("target is " + target)
            file_path = camera.capture(gphoto2.GP_CAPTURE_IMAGE)
            camera_file = camera.file_get(file_path.folder, file_path.name, gphoto2.GP_FILE_TYPE_NORMAL)
            camera_file.save(target)
            logger.info(image_hostName + image_luJing)
            files = {'file': open(target, 'rb')}
            r = requests.post(image_hostName + image_luJing, files=files, auth=(renzheng_name, renzheng_secret))
            logger.info("photoNum is " + str(photoNum))
            if r.status_code != 200:
                logger.info("take photo error,code is not 200")
                iot_reply_other('take_photo', '服务器未成功应答')
            else:
                iot_reply_ok('take_photo')
                logger.info("photo post success")
                photoNum = photoNum + 1
                HeartBeat['PhotoNum'] = photoNum
        else:
            iot_reply_other('take_photo',"未插入SD卡")
    except Exception as e:
        info = 'take_photo' + " error:" + str(e)
        iot_reply_error('take_photo', '相机未连接或未识别到')

def get_photoNum(mission_num):
    global photoNum
    if os.path.exists(sd_lujing+'/'+mission_num):
        photoNum=len(os.listdir(sd_lujing+'/'+mission_num))
        logger.info("current photoNum is "+str(photoNum))
        photoNum=photoNum+1
        logger.info("next photo index to "+str(photoNum))
    else:
        logger.info("not find photo in that mission")
        photoNum=1
        logger.info("change next photo index to 1")

def on_topic_message(topic, payload, qos, userdata):
    # 接受阿里云iot json数据包
    global id_command, NewFirmwareFileName, url
    global flag_ack
    global planFileName
    global flag_start
    global photoNum
    global mission_num
    global image_hostName
    global image_luJing
    global renzheng_name
    global renzheng_secret
    global sd_lujing
    global ack
    jieshou = json.loads(payload)
    if jieshou['Type'] == 'Command':
        if jieshou['CMD'] == 1:  # 起飞指令
            # flag_start=1
            the_connect.arducopter_arm()  # qifei
            # 等待解锁应答，如果解锁失败，则不发送“任务开始”命令
            if wait_for_ack(1):#解锁命令等待应答成功
                time.sleep(1)
                aa.command_long_send(the_connect.target_system, the_connect.target_component, cm.MAV_CMD_MISSION_START,
                                     0, 0, 0, 0, 0, 0, 0, 0)#开始任务
                if wait_for_ack(1):#开始任务命令等待应答成功
                    flag_start = 1  # 本来是应该放在读取飞机的状态，如果是巡航状态则置位该标志位，测试一起飞就
                    logger.info("起飞ing")
                    iot_reply_command(1,0,"起飞执行成功")
                else:
                    iot_reply_command(1,1,"解锁成功-起飞被拒绝")
            else:
                iot_reply_command(1, 1,"解锁被拒绝")
                # iot_reply_other("Command","refuse disarm")
        if jieshou['CMD'] == 2:  # 紧急降落
            id_command = 2
            flag_ack = 1
            flag_start = 0  # 结束任务，不拍照了
            #aa.command_long_send(the_connect.target_system,
                                 #the_connect.target_component,
                                 #cm.MAV_CMD_DO_PARACHUTE,
                                 #0,
                                 #cm.PARACHUTE_ENABLE,
                                 #0,
                                 #0,
                                 #0,
                                 #0,
                                 #0,
                                 #0)#开伞舱
            logger.info("降落ing")
        if jieshou['CMD'] == 3:  # 悬停指令
            logger.info("悬停ing")
            id_command = 3
            flag_ack = 1
        if jieshou['CMD'] == 4:  # 返航指令
            logger.info("返航ing")
            #the_connect.set_mode_rtl()#debug
            id_command = 4
            flag_ack = 1
        if jieshou['CMD'] == 5:  # 加锁指令
            logger.info("加锁ing")
            id_command = 5
            flag_ack = 1
        if jieshou['CMD'] == 6:  # 查询错误码指令
            id_command = 6
            flag_ack = 1
            logger.info("查询错误码ing")
    elif jieshou['Type'] == 'SendWayPoint':
        try:
            url = jieshou['url']
            planFileName = url.split('/')[-1]
            logger.info("xia fa hang xian:"+url)
            if os.path.exists("planfile/"+planFileName) is True:
                logger.info("planfile is exist")
            else:
                os.chdir('planfile/')
                planFileName = wget.download(url)
                os.chdir('../')
                # 拉取文件成功
                logger.info("get file")
            # 开始下发航线
            if xiaFangHangXian():
                iot_reply_ok('SendWayPoint')
                # if read_planfile():
                #     if plan_check():
                #         logger.info("check success")
                #         iot_reply_ok('check_line')
                #     else:
                #         logger.info("check error")
                #         iot_reply_other('check_line', 'check error')
                # else:
                #     iot_reply_other('check_line','read error')
            else:
                iot_reply_other('SendWayPoint','飞控未应答，请重启飞机')
        except ValueError:
            logger.info("planfile is not json")
            iot_reply_other('SendWayPoint',"航线文件格式错误，请检查航线文件")
        except Exception as e:# 拉取文件失败
            info = "get file failed: " + str(e)
            logger.info(info)
            iot_reply_error('SendWayPoint',e)
    elif jieshou['Type'] == 'MISSION':
        try:
            mission_num = str(jieshou['MissionID'])
            logger.info("mission num changed,new num is " + mission_num)
            if os.path.exists(sd_lujing +"/"+ mission_num) is False:
                os.mkdir(sd_lujing +"/"+ mission_num)
            HeartBeat['mission_id'] = mission_num
            with open('initial.ini', 'r') as f:
                write_json=f.read()
            writedata = json.loads(write_json)  # 将json转换为字典
            writedata['mission_num'] = mission_num
            write_json = json.dumps(writedata)
            with open('initial.ini', 'w') as f:
                f.write(write_json)
            logger.info("mission_num save success")
            get_photoNum(mission_num)
            iot_reply_ok('MISSION')
        except Exception as e:
            info="MISSION error" +str(e)
            logger.info(info)
            iot_reply_error('MISSION',e)
    elif jieshou['Type'] == 'read_flight_trajectory':
        # logger.info(jieshou)
        flag_start=0
        target = "trace/" + str(jieshou['mission_id']) + ".tr"
        if os.path.exists(target) is True:
            logger.info("trace file is exists and ready to post")
            try:
                out = '{\"data\":['
                with open(target, 'r') as f:
                    lines = f.readlines()
                for line in lines:
                    out = out + line[:-1] + ','
                out = out[:-1] + "]}"
                with open(target+'ace','w+') as f:
                    f.write(out)
                logger.info("target is "+target)
                files = {'file': open(target+'ace', 'rb')}
                r = requests.post(jieshou['url'], files=files)
                if r.status_code != 200:
                    logger.info(r.text)
                    raise Exception(json.loads(r.text)['error'])
                else:
                    logger.info("trace file post success")
                    iot_reply_ok('read_flight_trajectory')
            except Exception as e:
                info="read_flight_trajectory error" +str(e)
                logger.info(info)
                iot_reply_error('read_flight_trajectory',e)
        else:
            logger.info("trace file is not exists")
            iot_reply_error('read_flight_trajectory',"轨迹文件不存在，请核对任务号")
    elif jieshou['Type'] == 'set_iot_key':  # 设置三元组，修改init文件，下次启动时连接新的三元组
        try:
            with open('initial.ini', 'r') as f:
                write_json=f.read()
            logger.info(write_json)
            writedata = json.loads(write_json)  # 将json转换为字典
            writedata[whichserver]['product_key'] = jieshou['product_key']
            writedata[whichserver]['device_secret'] = jieshou['device_secret']
            write_json = json.dumps(writedata)
            with open('initial.ini', 'w') as f:
                f.write(write_json)
            logger.info("setting success")
            iot_reply_ok('set_iot_key')
        except Exception as e:
            info = "set_iot_key: " + str(e)
            logger.info(info)
            iot_reply_error('set_iot_key',e)
    elif jieshou['Type']=="set_device_firmware":#固件升级
        try:
            # 从服务器给定的位置下载固件文件
            #logger.info(jieshou)
            url = jieshou['url']
            wantfile = url.split('/')[-1]
            if os.path.exists("firmware/"+wantfile) is True:
                logger.info("firmware file is exist")
            else:
                NewFirmwareFileName = wget.download(url)
                logger.info("get new firmware file")
                logger.info("new firmware file is "+NewFirmwareFileName)
                # 保存固件文件到firmware目录下
                if os.path.exists('firmware') is not True:
                    os.mkdir('firmware')
                with open(NewFirmwareFileName,'r') as f:
                    fileContext = f.read()
                if 'import' in fileContext:
                    os.system('mv ' + NewFirmwareFileName + ' firmware/')
                    # 修改init文件
                    with open('initial.ini', 'r') as f:
                        init_json = f.read()
                    init_dict = json.loads(init_json)
                    init_dict['runfile'] = NewFirmwareFileName
                    init_json = json.dumps(init_dict)
                    with open('initial.ini', 'w') as f:
                        f.write(init_json)
                    logger.info("init file modify success")
                    iot_reply_ok('set_device_firmware')
                else:
                    logger.info(NewFirmwareFileName+" format error")
                    iot_reply_other('set_device_firmware','固件格式错误，请检查固件')
        except Exception as e:
            info = "device update error "+str(e)
            logger.info(info)
            iot_reply_error('set_device_firmware',e)
    elif jieshou['Type']=='take_photo':
        threading.Thread(target=take_a_photo,args=(jieshou,)).start()
    elif jieshou['Type'] == 'image_post':
        try:
            logger.info(jieshou)
            if 'user' not in jieshou:
                jieshou['user']=renzheng_name
            if 'password' not in jieshou:
                jieshou['password']=renzheng_secret
            if 'url' not in jieshou:
                jieshou['url']=image_hostName+image_luJing
            if 'ip' in jieshou:
                jieshou['url']="http://"+jieshou['ip']+":8080/"+image_luJing
            image_url_modify(jieshou['url'],jieshou['user'],jieshou['password'])
            iot_reply_ok('image_post')
        except Exception as e:
            iot_reply_error('image_post',e)
    elif jieshou['Type']=='read_SD_info':
        try:
            logger.info(jieshou)
            if os.path.exists(sd_lujing):
                os.system("du -h --exclude '.*' "+sd_lujing+">dubuffer")
                with open("dubuffer",'r') as f:
                    lines=f.read()
                iot_reply_ok_some('read_SD_info',lines)
                logger.info(lines)
            else:
                iot_reply_other('read_SD_info','SD卡未插入或未挂载')
        except Exception as e:
            iot_reply_error('read_SD_info',e)
    elif jieshou['Type']=='delete_photo':
        try:
            logger.info(jieshou)
            if os.path.exists(sd_lujing+"/"+jieshou['mission_id']):
                os.system("rm -rf "+sd_lujing+"/"+jieshou['mission_id'])
                iot_reply_ok('delete_photo')
            else:
                iot_reply_other('delete_photo','路径不存在，请检查')
        except Exception as e:
            iot_reply_error('delete_photo',e)
    elif jieshou['Type']=='soft_reset':
        try:
            logger.info(jieshou)
            iot_reply_ok("soft_reset")
            os.execl("/usr/bin/python3","python3","start.py")
        except Exception as e:
            iot_reply_error('soft_reset',e)
    elif jieshou['Type']=='arm_test':
        try:
            logger.info(jieshou)
            if jieshou['action']=='arm':
                the_connect.arducopter_arm()
                if wait_for_ack(1):
                    iot_reply_ok('arm_test')
                else:
                    iot_reply_other('arm_test','被飞控拒绝')
            elif jieshou['action']=='disarm':
                the_connect.arducopter_disarm()
                if wait_for_ack(1):
                    iot_reply_ok('arm_test')
                else:
                    iot_reply_other('arm_test','被飞控拒绝')
            else:
                iot_reply_other('arm_test','命令不支持，请重试')
        except Exception as e:
            iot_reply_other('arm_test',e)
    elif jieshou['Type']=='check_line':
        try:
            if read_planfile():
                if plan_check():
                    logger.info("check success")
                    iot_reply_ok('check_line')
                else:
                    logger.info("check error")
                    iot_reply_other('check_line','航向核对失败，请检查航线，并重新下发')
            else:
                iot_reply_other('check_line','读航线失败，请检查飞控')
                logger.info("read wp error")
        except Exception as e:
            iot_reply_other('parachute',e)
    else:
        logger.info(jieshou)




def dingshi_paiZhao():
    global flag_start
    global mission_num
    global HeartBeat
    global photoNum
    logger.info('Capturing image')
    if os.path.exists(sd_lujing+"/"+ mission_num) is False:
        os.mkdir(sd_lujing +"/"+ mission_num)
    while flag_start == 1:
        try:
            camera=gphoto2.Camera()
            camera.init()
            file_path = camera.capture(gphoto2.GP_CAPTURE_IMAGE)
            logger.info('Camera file path: {0}/{1}'.format(file_path.folder, file_path.name))
            target = sd_lujing+"/"+ mission_num + "/" + mission_num + "_" + str(photoNum) + ".jpg"
            logger.info('Copying image to ' + target)
            camera_file = camera.file_get(file_path.folder, file_path.name, gphoto2.GP_FILE_TYPE_NORMAL)
            camera_file.save(target)
            modify(target, HeartBeat['lat'], HeartBeat['lon'], HeartBeat['alt'], HeartBeat['yaw'], HeartBeat['roll'],HeartBeat['pitch'])
            photoNum = photoNum + 1
            HeartBeat['PhotoNum']=photoNum
            time.sleep(5)
            camera.exit()
        except Exception as e:
            logger.info("paizhao:"+str(e))
            time.sleep(1)


def post_image():
    global flag_start
    global photoNum
    image_name=mission_num+"_1.jpg"
    logger.info("post_image thread start")
    while flag_start == 1:
        try:
            while os.path.exists(sd_lujing +"/"+ mission_num + "/" + image_name) is False or int(image_name.split("_")[-1].split(".")[0])>=photoNum:
                logger.info("waiting for capture image " + image_name)
                if flag_start == 0:
                    break
                time.sleep(1)
            if flag_start == 0:
                break
            files={'file':open(sd_lujing+"/"+mission_num+"/"+image_name,'rb')}
            r = requests.post(image_hostName + image_luJing, files=files, auth=(renzheng_name, renzheng_secret))
            if r.status_code!=200:
                logger.info("post image err,status_code is "+str(r.status_code))
            else:
                logger.info("post image "+image_name+" success!")
                image_name=r.text
        except Exception as e:
            logger.info("image post err:"+str(e))
    logger.info("post_image thread end")


def trace_save():
    global flag_start
    global HeartBeat
    global mission_num
    if flag_start==1:
        target = "trace/" + mission_num + ".tr"#进程存，没有头尾
        if os.path.exists(target):
            with open(target, 'a') as f:
                f.write(str(json.dumps(HeartBeat))+ "\n")
                while flag_start == 1:
                    time.sleep(1)
                    f.write(str(json.dumps(HeartBeat))+ "\n")
        else:
            with open(target, 'w+') as f:
                f.write(str(json.dumps(HeartBeat))+ "\n")
                while flag_start == 1:
                    time.sleep(1)
                    f.write(str(json.dumps(HeartBeat))+ "\n")


def on_device_dynamic_register(rc,value,userdata):
    if rc==0:
        logger.info("dynamic register device success,rc:%d,value:%s"%(rc,value))
        #value里面就是devicesecret
        with open('initial.ini','r') as f:
            file_json=f.read()
        file_dict = json.loads(file_json)
        file_dict[whichserver]['device_secret']=value
        file_json = json.dumps(file_dict)
        with open('initial.ini','w') as f:
            f.write(file_json)
    else:
        logger.info("dynamic register device fail,rc:%d,value:%s"%(rc,value))

if __name__ == "__main__":
    # 创建文件夹
    if os.path.exists('log') is False:
        os.mkdir('log')
    if os.path.exists('trace') is False:
        os.mkdir('trace')
    # if os.path.exists('image') is False:
    #     os.mkdir('image')
    if os.path.exists('planfile') is False:
        os.mkdir('planfile')
    # 创建Log文件，配置log，每次程序启动，用程序启动时间命令log文件
    logger = logging.getLogger(__name__)
    logger.setLevel(level=logging.INFO)
    formatter = logging.Formatter('%(levelname)s-%(message)s')
    # logfileName = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
    logfileName = datetime.datetime.now().strftime("%Y%m%d")
    logfileName = "log/" + logfileName + ".log"
    file_log = logging.FileHandler(logfileName, mode='a')
    file_log.setLevel(logging.INFO)
    file_log.setFormatter(formatter)
    logger.addHandler(file_log)
    console = logging.StreamHandler()
    console.setLevel(logging.INFO)
    console.setFormatter(formatter)
    #logger.addHandler(console)
    logger.info("start logging")
    resetNet()
    os.system("sudo route add default ppp0")
    # 创建连接 打开连接飞控的串口
    the_connect = mavutil.mavlink_connection("/dev/feikong_uart", baud=57600)
    aa = cm.MAVLink(the_connect)
    # 阿里云初始化
    if len(sys.argv) >1:
        flag_start=1
    else:
        flag_start=0
    # whichserver = sys.argv[1] #如果从start.py启动
    whichserver = 'hejing'
    # whichserver = 'wangbo'
    pd_key,pd_secret,dv_secret,dv_name= read_init(whichserver)
    logger.info("current mission num is "+str(mission_num))
    get_photoNum(mission_num)
    #根据任务号去找目录下照片的数量
    lk = linkkit.LinkKit(
        host_name="cn-shanghai",
        product_key=pd_key,
        device_name=dv_name,
        device_secret=dv_secret,
        product_secret=pd_secret
    )
    lk.config_mqtt(port=1883, protocol="MQTTv311", transport="TCP",
                secure="TLS", keep_alive=60, clean_session=False,
                max_inflight_message=1, max_queued_message=0,
                auto_reconnect_min_sec=5,
                auto_reconnect_max_sec=5,
                cadata=None)
    if dv_secret=="":#如果为空，代表还没有动态注册
        lk.on_device_dynamic_register = on_device_dynamic_register
    else:#已经动态注册过，直接调用
        pass
    HeartBeat['device_id'] = dv_name
    HeartBeat['UAV_Firmware'] = __file__.split('/')[-1].split('.')[0]
    logger.info("firmware version is " + HeartBeat['UAV_Firmware'])
    lk.on_connect = on_connect  # 设备连接云端成功后会通过on_connect回调函数通知用户
    lk.on_disconnect = on_disconnect  # 连接断开会通过on_disconnect 回调通知用户
    lk.on_publish_topic = on_publish_topic  # 发送消息成功与否回调函数
    lk.connect_async()  # 启动连接

    # 线程创建
    threading.Thread(target=dingshi).start()
    threading.Thread(target=heartBeatRecord).start()
    threading.Thread(target=heartBeatSend).start()
    threading.Thread(target=read_mavlink).start()
    # t2.start()


    # # 打开搜索卫星功能
    # logger.info("satallite message interval start")
    # the_connect.mav.command_long_send(the_connect.target_system,
    #                                   the_connect.target_component,
    #                                   cm.MAV_CMD_SET_MESSAGE_INTERVAL,
    #                                   0,
    #                                   cm.MAVLINK_MSG_ID_GPS2_RAW,
    #                                   2000000,
    #                                   0,
    #                                   0,
    #                                   0,
    #                                   0,
    #                                   2)
    # wait_for_ack(3)
    # logger.info("send gps command success")
    while True:
        if flag_start == 1:
            t3 = threading.Thread(target=dingshi_paiZhao)
            t3.start()
            t4 = threading.Thread(target=post_image)
            t4.start()
            t5 = threading.Thread(target=trace_save)
            t5.start()
            t3.join()
            t4.join()
            t5.join()
        time.sleep(1)
    t1.join()
    t2.join()
