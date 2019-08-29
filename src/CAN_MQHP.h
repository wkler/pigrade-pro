/*
 * CAN_MQHP.h
 *
 *  Created on: 2019年8月13日
 *      Author: wkler
 */

#ifndef CAN_MQHP_H_
#define CAN_MQHP_H_

#define NODE_ID_BROADCAST     0xff

/* topic type definations */
#define TOPIC_HOST_GLOBAL_CMD                0x07FF
#define TOPIC_HOST_IMG_STREAM_PANEL          0x05FF
#define TOPIC_HOST_IMG_STREAM_PICO           0x05FE

#define TOPIC_PANEL_CMD_RESP                 0x7FE
#define TOPIC_PANEL_RH                       0x2FE
#define TOPIC_PANEL_TMP                      0x2FF
#define TOPIC_PANEL_SYSLOG                   0x1FF

#define TOPIC_ALL_PING                       0x03FF
#define TOPIC_ALL_PONG                       0x03FE


/* HOST command type definations */
#define CMD_RESET                               0x00
#define CMD_CONFIG_IMG_INFO1                    0x01
#define CMD_CONFIG_IMG_INFO2                    0x02
#define CMD_START_SEND_IMG                      0x03
#define CMD_STOP_SEND_IMG                       0x04
#define CMD_EXE_UPGRADE                         0x05


#define CMD_CHECK_VER_PANEL                     0x07
#define CMD_CHECK_VER_PICO                      0x08
#define CMD_START_ACQ                           0x09
#define CMD_STOP_ACQ                            0x0A
#define CMD_EXE_TDC_CALI                        0x0B
#define CMD_SRCH_PICO_NUM_ALL                   0x0C
#define CMD_SRCH_PICO_NUM_ACTIVE                0x0D
#define CMD_SET_PICO_NUM_ACTIVE                 0x0E
#define CMD_EXEC_PING_PICO                      0x0F

/* SLAVE response message to host */

#define RESP_RECV_OK                            0x70
#define RESP_RECV_ERR                           0x71
#define RESP_IMG_PANEL_CHECK_OK                 0x72
#define RESP_IMG_PANEL_CHECK_ERR                0x73
#define RESP_IMG_PANEL_TRANS_REPORT             0x74
#define RESP_IMG_PICO_CHECK_OK                  0x75
#define RESP_IMG_PICO_CHECK_ERR                 0x76
#define RESP_IMG_PICO_TRANS_REPORT              0x77
#define RESP_PING_PICO                          0x78





void CAN_MQHP_task(void const * argument);
void upgrade_task(void const * argument);

#endif /* CAN_MQHP_H_ */
