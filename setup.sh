#!/bin/sh

sudo apt-get install vim -y
sudo service ssh start
sudo pip3 install pymavlink
sudo apt-get install libgphoto2-dev -y
sudo apt-get install gphoto2 -y
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

cd ~/ccssoft/
sudo cp rc.local /etc/rc.local



sudo sh -c 'echo "dtoverlay=pi3-disable-bt">>/boot/config.txt'
sudo sh -c 'echo "force_turbo=1">>/boot/config.txt'
sudo systemctl disable hciuart
