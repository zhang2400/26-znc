#!/bin/bash

ip="10.149.183.169"
#ip="192.168.188.169"
#ip="192.168.43.253"
username="loongson"
# 上传到开发板并运行
echo "Running :ssh $username@$ip sudo pkill cv"
ssh -t $username@$ip "sudo pkill cv"
sleep 0.5
echo "Running :ssh $username@$ip ./cv"
ssh -t $username@$ip "sudo ./cv"
