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
flag_start = 0
mission_num = "2003301112"
planFileName = 'test.plan'
#url = "http://47.92.114.156:8080/filemicroservice/imageRecognition"  # 微服务的路径
url = "http://39.99.240.145:8080/filemicroservice/imageRecognition"  # 微服务的路径
photoNum=1
image_hostName = "http://47.92.112.190:8080/"  # 微服务的路径
image_luJing="filemicroservice/imageRecognition"
renzheng_name="uav"
renzheng_secret="123456"


# 心跳json包样式
HeartBeat = {'Type': 'HeartBeat',
             'base_mode': 4,
             'custom_mode': 0,
             'mission_id': mission_num,
             'PhotoNum': 1,
             'system_status': 0,
             'battery_remaining': 90,
             'airspeed': 15,
             'groundspeed': 20,
             'lat': 0,
             'lon': 0,
             'alt': 20,
             'yaw': 0,
             'roll': 0,
             'pitch': 0,
             'UAV_Type': 'P330',
             'UAV_Firmware': 'DF_20',
             'device_id':"device1",
             'time':'2020-05-17 10:39:20',
             'image_url':image_hostName+image_luJing
             }


def read_init(whichserver):
    global renzheng_name
    global renzheng_secret
    global image_luJing
    global image_hostName
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
            logger.info("read image url succes")
            logger.info("image_url is "+image_hostName+image_luJing)
            HeartBeat['image_url']=image_hostName+image_luJing
        except:
            logger.info("read image url error")
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
    image_byte = piexif.dump(image_dict)
    piexif.insert(image_byte, filename)


def xiaFangHangXian():
    global request
    global ack
    global planFileName
    global flag_start

    # 清除原有的mission
    the_connect.mav.mission_clear_all_send(the_connect.target_system, the_connect.target_component)
    while ack == 0:
        the_connect.mav.mission_clear_all_send(the_connect.target_system, the_connect.target_component)
        logger.info("clear: waiting for ack")
        time.sleep(0.5)
    ack = 0
    # 读取plan文件内容
    planfile = open("planfile/"+planFileName, mode='r')
    filedata = planfile.read()
    mis_dict = json.loads(filedata)
    length = len(mis_dict['mission']['items'])
    logger.info("waypoint num is %d" % length)
    # 下发航线个数
    the_connect.mav.mission_count_send(the_connect.target_system, the_connect.target_component, length)
    while request == 0:
        logger.info("waiting for count reply")
        time.sleep(0.1)
    request = 0
    # 逐个的传
    logger.info("start to send waypoint")
    for i in range(length):
        logger.info("sending waypoint %d" % i)
        the_connect.mav.mission_item_int_send(the_connect.target_system, the_connect.target_component, i,
                                              mis_dict['mission']['items'][i]['frame'],
                                              mis_dict['mission']['items'][i]['command'],
                                              mis_dict['mission']['items'][i]['doJumpId'],
                                              mis_dict['mission']['items'][i]['autoContinue'],
                                              mis_dict['mission']['items'][i]['params'][0] * 1.0,
                                              mis_dict['mission']['items'][i]['params'][1] * 1.0,
                                              mis_dict['mission']['items'][i]['params'][2] * 1.0,
                                              0,
                                              int(mis_dict['mission']['items'][i]['params'][4] * 10000000),
                                              int(mis_dict['mission']['items'][i]['params'][5] * 10000000),
                                              mis_dict['mission']['items'][i]['params'][6] * 1.0,
                                              )
        while request == 0 and i != (length - 1):
            logger.info("waiting for item reply")
            time.sleep(0.1)
        request = 0
    while ack == 0:
        logger.info("waiting for ack")
        time.sleep(0.5)
    logger.info("mission send over")
    planfile.close()


def read_mavlink():  # 读mavlink数据放入心跳json包
    global request
    global ack
    while True:
        the_connect.recv_match(blocking=True)
        #print(the_connect.messages)
        #print(the_connect.mavlink10())
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
                elif sub_mode==5:
                    HeartBeat['base_mode']=13#返航中
                    flag_start=0
                elif sub_mode==6:
                    HeartBeat['base_mode']=8#降落中
        except:
            pass
        try:
            HeartBeat['battery_remaining'] = the_connect.messages['SYS_STATUS'].battery_remaining
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
            global flag_ack
            if flag_ack == 1:
                logger.info("command is %d result is %d\n" % (command, result))
                global flag_connect
                # result=0#debug
                if result == 0:  # 成功
                    # 发送成功json到阿里云
                    flag_start=1
                    msg = {'Type': 'Command', 'CMD': id_command, 'Status': 'ok', 'result': 0,'device_id':dv_name}
                    if 1 == flag_connect:
                        rc_sub, mid_sub = lk.publish_topic(lk.to_full_topic("user/update"), json.dumps(msg))
                else:  # 失败
                    # 发送失败json到阿里云
                    msg = {'Type': 'Command', 'CMD': id_command, 'Status': 'error', 'result': result,'device_id':dv_name}
                    if 1 == flag_connect:
                        rc_sub, mid_sub = lk.publish_topic(lk.to_full_topic("user/update"), json.dumps(msg))
                flag_ack = 0
        except:
            pass
        try:
            seq = the_connect.messages['MISSION_REQUEST_INT'].seq
            request = 1
            del the_connect.messages['MISSION_REQUEST_INT']
        except:
            pass
        try:
            seq = the_connect.messages['MISSION_REQUEST'].seq
            request = 1
            del the_connect.messages['MISSION_REQUEST']
        except:
            pass
        try:
            seq = the_connect.messages['MISSION_ACK'].type
            ack = 1
            del the_connect.messages['MISSION_ACK']
        except:
            pass
        the_connect.messages.clear()

def find_process(command):
    mudiLine=None
    os.system("ps -ax>buffer")
    with open('buffer','r') as f:
        lines=f.readlines()
        for line in lines:
            if line.find(command)!=-1:
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
def find_ppp0():
    os.system("ifconfig|grep ppp0>findppp0")
    with open('findppp0','r') as f:
        lines=f.readlines()
        for line in lines:
            if line.find('ppp0')!=-1:
                print("find ppp0")
                os.system('rm findppp0')
                return True
    print("not find ppp0")
    os.system('rm findppp0')
    return False

def resetNet():
    if find_process("pppd")==0:
        logger.info("net error...start reset net")
        os.system("sudo route del default")
        time.sleep(1)
        err_count=0
        while find_process("pppd")==0:
            os.system("sudo quectel-pppd.sh")
            time.sleep(5)
            logger.info("reset net...")
            print("reset net....")
        while find_ppp0() is False:
            logger.info("waiting for ppp0 ...")
            print("waiting for ppp0 ...")
            err_count=err_count+1
            if err_count>5:
                return 0
            time.sleep(1)
        os.system("sudo route add default dev ppp0")
        time.sleep(1)
        os.system("sudo phddns restart")
        time.sleep(2)
    else:
        # logger.info("net is working")
        pass
        
# 定时线程
def dingshi():
    err_count=0
    global flag_connect
    global HeartBeat
    while True:
        # 定时上传心跳包
        resetNet()
        HeartBeat['time']=str(datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S"))
        #print(HeartBeat)
        try:
            if 1 == flag_connect:
                rc_sub, mid_sub = lk.publish_topic(lk.to_full_topic("user/update"), json.dumps(HeartBeat))
                if rc_sub!=0:
                    err_count=err_count+1
                else:
                    err_count=0
                if err_count>10:
                    err_count=0
                    raise Exception("time out")
        except Exception as e:
            logger.info("net error reboot network:"+str(e))
            # os.system("sudo reboot")
        time.sleep(1)



# 阿里云的各种回调函数
def on_connect(session_flag, rc, userdata):
    logger.info("on_connect:%d,rc:%d,userdata:" % (session_flag, rc))
    # 订阅主题
    rc, mid = lk.subscribe_topic(lk.to_full_topic("user/get"))
    lk.on_subscribe_topic = on_subscribe_topic  # 订阅成功回调函数
    lk.on_topic_message = on_topic_message  # 收到消息回调函数
    global flag_connect
    flag_connect = 1
    pass


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

def iot_reply_other(title,other):#回复不支持当前指令等
    msg = {'Type': title, 'Status': 'error', 'Details': other,'device_id': dv_name}
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
    jieshou = json.loads(payload)
    if jieshou['Type'] == 'Command':
        if jieshou['CMD'] == 1:  # 起飞指令
            the_connect.arducopter_arm()#qifei
            time.sleep(1)
            aa.command_long_send(the_connect.target_system, the_connect.target_component,cm.MAV_CMD_MISSION_START, 0, 0, 0, 0, 0, 0, 0, 0)
            id_command = 1
            flag_ack = 1
            flag_start = 1  # 本来是应该放在读取飞机的状态，如果是巡航状态则置位该标志位，测试一起飞就
            logger.info("起飞ing")
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
            xiaFangHangXian()
            iot_reply_ok('SendWayPoint')
        except ValueError:
            logger.info("planfile is not json")
            iot_reply_other('SendWayPoint',"planfile is not json")
        except Exception as e:# 拉取文件失败
            info = "get file failed: " + str(e)
            logger.info(info)
            iot_reply_error('SendWayPoint',e)
    elif jieshou['Type'] == 'MISSION':
        try:
            mission_num = str(jieshou['MissionID'])
            # os.system("echo "+str(mission_num)+" >mn")
            logger.info("mission num changed,new num is " + mission_num)
            if os.path.exists("image/" + mission_num) is False:
                os.mkdir("image/" + mission_num)
            HeartBeat['mission_id'] = mission_num
            iot_reply_ok('MISSION')
        except Exception as e:
            info="MISSION error" +str(e)
            logger.info(info)
            iot_reply_error('MISSION',e)
    elif jieshou['Type'] == 'read_flight_trajectory':
        # print(jieshou)
        flag_start=0
        if os.path.exists("trace/" + str(jieshou['mission_id']) + ".trace") is True:
            logger.info("trace file is exists and ready to post")
            try:
                files = {'file': open("trace/" + str(jieshou['mission_id']) + ".trace", 'rb')}
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
            iot_reply_error('read_flight_trajectory',"trace file is not exists")
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
            #print(jieshou)
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
                    iot_reply_other('set_device_firmware','format error')
        except Exception as e:
            info = "device update error "+str(e)
            logger.info(info)
            iot_reply_error('set_device_firmware',e)
    elif jieshou['Type']=='take_photo':
        try:
            camera = gphoto2.Camera()
            camera.init()
            target="image/"+jieshou['mission_id']+"/"+jieshou['mission_id']+"_"+str(photoNum)+".jpg"
            logger.info("target is "+target)
            file_path=camera.capture(gphoto2.GP_CAPTURE_IMAGE)
            camera_file=camera.file_get(file_path.folder,file_path.name,gphoto2.GP_FILE_TYPE_NORMAL)
            camera_file.save(target)
            # modify(target,HeartBeat['lat'],HeartBeat['lon'],HeartBeat['alt'],HeartBeat['yaw'],HeartBeat['roll'],HeartBeat['pitch'])
            logger.info(image_hostName+image_luJing)
            files = {'file':open(target,'rb')}
            r = requests.post(image_hostName + image_luJing, files=files, auth=(renzheng_name, renzheng_secret))
            logger.info("photoNum is "+str(photoNum))
            if r.status_code !=200:
                logger.info("take photo error,code is not 200")
                iot_reply_other('take_photo','status_code is not 200')
            else:
                iot_reply_ok('take_photo')
                logger.info("photo post success")
                photoNum=photoNum+1
                HeartBeat['PhotoNum'] = photoNum
        except Exception as e:
            info = 'take_photo'+" error:"+str(e)
            logger.info(info)
            iot_reply_error('take_photo',e)
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
    else:
        logger.info(jieshou)

def on_publish_topic(mid, userdata):  # 发送消息成功与否回调函数
    logger.info("on_publish_topic mid:%d" % mid)


def dingshi_paiZhao():
    global flag_start
    global mission_num
    global HeartBeat
    global photoNum
    logger.info('Capturing image')
    if os.path.exists("image/" + mission_num) is False:
        os.mkdir("image/" + mission_num)
    while flag_start == 1:
        try:
            camera=gphoto2.Camera()
            camera.init()
            file_path = camera.capture(gphoto2.GP_CAPTURE_IMAGE)
            logger.info('Camera file path: {0}/{1}'.format(file_path.folder, file_path.name))
            target = "image/" + mission_num + "/" + mission_num + "_" + str(photoNum) + ".jpg"
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
            while os.path.exists("image/" + mission_num + "/" + image_name) is False or image_name.split("_")[-1].split(".")[0]>=photoNum:
                logger.info("waiting for capture image " + image_name)
                if flag_start == 0:
                    break
                time.sleep(1)
            if flag_start == 0:
                break
            files={'file':open("image/"+mission_num+"/"+image_name,'rb')}
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
    with open("trace/" + mission_num + ".trace", 'w+') as f:
        f.write('{\"data\":[')
        if flag_start==1:
            f.write(str(json.dumps(HeartBeat)))
            while flag_start == 1:
                time.sleep(1)
                f.write(",")
                f.write(str(json.dumps(HeartBeat)))
        f.write("]}")


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
    if os.path.exists('image') is False:
        os.mkdir('image')
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
    try:
        #camera = gphoto2.Camera()
        #camera.init()
        logger.info("camera init success")
    except:
        logger.info("camera init failed")
    # if os.path.exists("mn") is True:
    #    with open("mn") as f:
    #        mission_num=f.read()
    logger.info("current mission num is "+str(mission_num)) 
    HeartBeat['mission_id'] = mission_num
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
    pd_key,pd_secret,dv_secret,dv_name= read_init(whichserver)

    lk = linkkit.LinkKit(
        host_name="cn-shanghai",
        product_key=pd_key,
        device_name=dv_name,
        device_secret=dv_secret,
        product_secret=pd_secret
    )
    if dv_secret=="":#如果为空，代表还没有动态注册
        lk.on_device_dynamic_register = on_device_dynamic_register
    else:#已经动态注册过，直接调用
        pass
    HeartBeat['device_id'] = dv_name
    HeartBeat['UAV_Firmware'] = __file__.split('/')[-1].split('.')[0]
    logger.info("firmware version is " + HeartBeat['UAV_Firmware'])
    lk.on_connect = on_connect  # 设备连接云端成功后会通过on_connect回调函数通知用户
    lk.on_disconnect = on_disconnect  # 连接断开会通过on_disconnect 回调通知用户
    lk.connect_async()  # 启动连接
    lk.on_publish_topic = on_publish_topic  # 发送消息成功与否回调函数
    # 线程创建
    t1 = threading.Thread(target=dingshi)
    t2 = threading.Thread(target=read_mavlink)
    t1.start()
    t2.start()
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
