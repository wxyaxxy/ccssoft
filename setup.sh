#!/bin/sh

sudo apt-get install vim -y
sudo service ssh start
sudo pip3 install pymavlink
sudo apt-get install libgphoto2-dev -y
sudo apt-get install gphoto2 -y
#如果gphoto2 持续安装失败，则先执行update image source，更新镜像源
sudo pip3 install gphoto2
sudo pip3 install wget
sudo pip3 install piexif
sudo pip3 install aliyun-iot-linkkit

echo "-----update image source----"
sudo cp /etc/apt/sources.list /etc/apt/sources.list.bak
sudo cp /etc/apt/sources.list.d/raspi.list /etc/apt/sources.list.d/raspi.list.bak
sudo cp ~/ccssoft/sources.list /etc/apt/sources.list
sudo cp ~/ccssoft/raspi.list /etc/apt/sources.list.d/raspi.list
sudo apt-get update
echo "-----wait for image update------"
sudo apt-get install ppp -y

mkdir ~/ccssoft/wangbo
cp ~/ccssoft/linux-ppp-scripts_V1.2.zip ~/wangbo/
cd ~/ccssoft/wangbo
unzip linux-ppp-scripts_V1.2.zip
cd ~/ccssoft/linux-ppp-scripts_V1.2/linux-ppp-scripts/
chmod +x quectel-pppd.sh
chmod +x quectel-ppp-kill
sudo cp quectel-chat-connect /etc/ppp/peers
sudo cp quectel-chat-disconnect /etc/ppp/peers
sudo cp quectel-ppp /etc/ppp/peers
sudo cp quectel-pppd.sh /bin/
#更新/bin/quectel-pppd.sh中端口，/dev/ttyUSB4
cd ~/ccssoft/


#关闭蓝牙
sudo sh -c 'echo "dtoverlay=pi3-disable-bt">>/boot/config.txt'
sudo sh -c 'echo "force_turbo=1">>/boot/config.txt'
sudo systemctl disable hciuart

#检查rc.local文件内容是否合适
#检查start.py文件内容是否合适
#检查initial.ini文件内容是否合适，更新新的三元组到initial.ini中
sudo cp rc.local /etc/rc.local

#创建firmware目录，将固件放入其中
#检查固件文件是否合适
mkdir /ccssoft/firmware

#加入/dev/feikong_uart
#在etc/udev/rules.d/找到xxx.rules文件，在最后增加以下一行
#KERNEL=="ttyUSB*", ATTRS{idVendor}=="1a86", ATTRS{idProduct}=="7523", MODE:="0777", SYMLINK+="feikong_uart"

#测试一下，是否能跑起来
sudo python3 start.py




