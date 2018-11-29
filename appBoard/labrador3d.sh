#!/bin/bash

#[this will be run every time labrador is turned on]

# =============================================
# PORT NUMBER TO CONNECT TO CENTRAL [EDIT THIS]
# select different number for each board
# (from 2000 to 5000, step 200, with no gaps)
PORT=2000

# =============================================
#INTERNAL PARAMETERS
IP=10.1.1.250 # with switch
# for tests, use IP of central computer (ifconfig)
PASS=citi
# password of labrador board
IMAGE=001
IMAGE2=compac
# to perform with test images, make a copy inside this dir: test-image/test.jpg
# ==============================================

echo $PASS | sudo -S modprobe owl_camera
sudo modprobe ov5640 rear=1

while :; do

./bin/01_receive $IP $PORT
read PRESET < "com1.txt"
rm com1.txt

./bin/02_picture $IP $(($PORT + 10))

if [ -e "com2.txt" ]
then
  cp test-image/test.jpg "$IMAGE.jpg"
  rm com2.txt
else
  mplayer -vo jpeg -frames 2 tv://
  mv 00000002.jpg "$IMAGE.jpg"
fi

./bin/03_process $IMAGE $PRESET

./bin/04_send $IP $(($PORT + 20)) $IMAGE $IMAGE2

done
