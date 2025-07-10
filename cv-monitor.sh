#!/bin/bash

# 用法
# scp -O cv-monitor.sh loongson@192.168.5.72:/home/loongson/
# scp -O cv-monitor.service loongson@192.168.5.72:/home/loongson/
# 在开发板上运行以下命令
# sudo chmod +x cv-monitor.sh
# sudo chmod +x cv-monitor.service
# sudo cp cv-monitor.service /etc/systemd/system/
# sudo systemctl enable cv-monitor.service
# sudo systemctl start cv-monitor.service

log_kmsg() {
    echo "<6>${LOG_TAG} $1" > /dev/kmsg
}

unbind_fb_console() {
    for i in /sys/class/vtconsole/vtcon*; do
        name=$(cat "$i/name")
        if echo "$name" | grep -iq "frame buffer"; then
            echo 0 > "$i/bind"
        fi
    done
}

sleep 10

log_kmsg "Ready"
log_kmsg "Ready"
log_kmsg "Ready"
log_kmsg "Ready"
log_kmsg "Ready"
log_kmsg "Ready"
log_kmsg "Ready"
log_kmsg "Ready"
log_kmsg "Ready"
log_kmsg "Ready"
log_kmsg "Ready"
log_kmsg "Ready"

GPIO_NUM=16
PROCESS_NAME="cv"
PROCESS_PATH="/home/loongson/cv"
LOG_TAG="[cv_monitor]"
GPIO_PATH="/sys/class/gpio/gpio${GPIO_NUM}/value"
GPIO_DIR="/sys/class/gpio/gpio${GPIO_NUM}/direction"

# 初始化GPIO（如果未导出）
if [ ! -e "$GPIO_PATH" ]; then
    echo "$GPIO_NUM" > /sys/class/gpio/export
    echo "in" > "$GPIO_DIR"
fi

unbind_fb_console
while true; do

    # 检查GPIO是否为低电平
    if [ "$(cat "$GPIO_PATH")" = "0" ]; then
        # 等待1秒以避免抖动
        sleep 1
        # 再次检查GPIO状态
        if [ "$(cat "$GPIO_PATH")" = "0" ]; then
            # 如果仍然是低电平，重启进程
            log_kmsg "GPIO $GPIO_NUM low detected, restarting $PROCESS_NAME..."
            killall -q "$PROCESS_NAME"
            sleep 1
            "$PROCESS_PATH" >/dev/null 2>&1 &
            log_kmsg "$PROCESS_NAME restarted."
        fi
    fi
    sleep 1
done
