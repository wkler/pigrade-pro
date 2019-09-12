# CAN调试测试 使用 can-utils 工具

## 参考资料： 
https://github.com/linux-can/can-utils
https://www.kernel.org/doc/Documentation/networking/can.txt
https://elinux.org/CAN_Bus
https://packages.debian.org/sid/can-utils
https://blog.csdn.net/yuanlulu/article/details/7220060
http://lxr.linux.no/linux+v2.6.34/Documentation/networking/can.txt
http://www.brownhat.org/docs/socketcan/llcf-api.html#SECTION00051000000000000000
https://github.com/linux-can/can-utils

## 获取socketCAN帮助信息
ip link set can0 type can help
Usage: ip link set DEVICE type can
        [ bitrate BITRATE [ sample-point SAMPLE-POINT] ] | 
        [ tq TQ prop-seg PROP-SEG phase-seg1 PHASE-SEG1
          phase-seg2 PHASE-SEG2 [ sjw SJW ] ]

        [ dbitrate BITRATE [ dsample-point SAMPLE-POINT] ] | 
        [ dtq TQ dprop-seg PROP_SEG dphase-seg1 PHASE-SEG1
          dphase-seg2 PHASE-SEG2 [ dsjw SJW ] ]

        [ loopback { on | off } ]
        [ listen-only { on | off } ]
        [ triple-sampling { on | off } ]
        [ one-shot { on | off } ]
        [ berr-reporting { on | off } ]
        [ fd { on | off } ]
        [ fd-non-iso { on | off } ]
        [ presume-ack { on | off } ]

        [ restart-ms TIME-MS ]
        [ restart ]

        Where: BITRATE  := { 1..1000000 }
                  SAMPLE-POINT  := { 0.000..0.999 }
                  TQ            := { NUMBER }
                  PROP-SEG      := { 1..8 }
                  PHASE-SEG1    := { 1..8 }
                  PHASE-SEG2    := { 1..8 }
                  SJW           := { 1..4 }
                  RESTART-MS    := { 0 | NUMBER }

## 查看can socket的状态信息
ip -details link show can0

## 查看can socket的统计信息
ip -details -statistics link show can0

## 启动can
ifconfig can0 up
或者
ip link set up can0

## 设置can通信波特率时序
ip link set can0 type can tq 125 prop-seg 6 phase-seg1 7 phase-seg2 2 sjw 1 
或者
ip link set can0 type can bitrate 125000
ip link set can0 type can bitrate 1000000

## 停止can
ifconfig can0 down
或者
ip link set down can0

## 重新启动can模块
ip link set can0 type can restart

## 设置can自动重启时间
ip link set can0 type can restart-ms 100

## 设置can波特率同时开启can
ip link set can0 up type can bitrate 1000000

## 发送单帧
./cansend can0 123#DEADBEEF

## 发送随机帧序列
./cangen can0 -g 777 -I 42A -L 1 -D i -v -v
或者
./cangen can0 -g 555 -I i -L 8 -D i -v -v

## 接受can帧
./cansniffer can0

## 记录收到的所有can帧序列 记录错误信息
./candump -l any,0:0,#FFFFFFFF

## 使用cantool工具持续发送
./cantoolexe -p 0 -b 1000 -f 500 -t 100 -s -I -g 



data:
ff 03 00 00 08 00 00 00 11 ff 70 69 6e 67 77 fe


root@SysHost:~ kill -9 1673 
root@SysHost:~ netstat -ap | grep 'pigrade'

echo 1000 > /sys/class/net/can0/tx_queue_len
cat /sys/class/net/can0/tx_queue_len
ulimit -s 16384 

## run as a server.  daemon mode
ulimit -s 16384 
echo 1000 > /sys/class/net/can0/tx_queue_len
echo 100 > /sys/class/net/can0/tx_queue_len
ip link set can0 up type can bitrate 1000000
root@SysHost:~ ./pigrade -p 8898 -d
## run as a client.  front mode
ulimit -s 16384
root@SysHost:~ ./pigrade -c 192.168.101.164 -p 8898 -I ~/image.bin -t panel
ip -details -statistics link show can0


Examples:

ifconfig can0 txqueuelen 1000

or

ip link set can0 txqueuelen 1000

ip link set can0 type can tq 75 prop-seg 6 phase-seg1 7 phase-seg2 2 sjw 1 





Can Open 失败

未运行 参数设置命令
ip link set CanX up type can bitrate 500000
Can 错误处理

出现太多的错误 设备可能计入总线关闭状态，给canrestart-ms 设置一个非零值，可以开启总线关闭自动恢复功能
ip link set CanX type can restart-ms 100
————————————————
版权声明：本文为CSDN博主「jionfull」的原创文章，遵循 CC 4.0 BY-SA 版权协议，转载请附上原文出处链接及本声明。
原文链接：https://blog.csdn.net/jionfull/article/details/82490625



https://blog.csdn.net/lybhit/article/details/78663347

ip link set can0 type can tq 125 prop-seg 6 phase-seg1 7 phase-seg2 2 sjw 1

ip link set can0 type can tq 25 prop-seg 23 phase-seg1 8 phase-seg2 8 sjw 1

ip link set can0 type can tq 50 prop-seg 11 phase-seg1 4 phase-seg2 4 sjw 1

ifconfig can0 txqueuelen 1000
ip link set down can0
ip link set can0 type can tq 50 prop-seg 7 phase-seg1 7 phase-seg2 5 sjw 1 
ip link set up can0



ulimit -s 16384 
ip -details -statistics link show can0
ifconfig can0 txqueuelen 1000
ip link set down can0
ip link set can0 type can tq 50 prop-seg 7 phase-seg1 7 phase-seg2 5 sjw 1 
ip link set up can0
./pigrade -p 8898
./pigrade -c 192.168.101.164 -p 8898 -I ~/image.bin
./pigrade -c 192.168.101.164 -p 8898 -I ~/image.bin -t panel


ulimit -s 16384 
ip -details -statistics link show can1
ifconfig can1 txqueuelen 1000
ip link set down can1
ip link set can1 type can tq 50 prop-seg 7 phase-seg1 7 phase-seg2 5 sjw 1 
ip link set up can1
./pigrade -p 8898
./pigrade -c 192.168.101.164 -p 8898 -I ~/image.bin
./pigrade -c 192.168.101.164 -p 8898 -I ~/image.bin -t panel

<can_id>##<flags>{data}
./cansend can1 213##311223344