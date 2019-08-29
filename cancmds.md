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

## run as a server.  daemon mode
root@SysHost:~ ./pigrade -p 8898 -d
## run as a client.  front mode
root@SysHost:~ ./pigrade -c 192.168.101.164 -p 8898 -d -I ~/image.bin -t panel

