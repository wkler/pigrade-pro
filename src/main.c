/*                                                                                                                                             
 * pigrade - A simple CAN-over-IP gateway
 * Copyright (C) 2016 Matthias Kruk                                                                                                            
 *                                                                                                                                             
 * pigrade is free software; you can redistribute it and/or modify                                                                               
 * it under the terms of the GNU General Public License as published                                                                           
 * by the Free Software Foundation; either version 3, or (at your                                                                              
 * option) any later version.                                                                                                                  
 *                                                                                                                                             
 * pigrade is distributed in the hope that it will be useful, but                                                                                
 * WITHOUT ANY WARRANTY; without even the implied warranty of                                                                                  
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU                                                                           
 * General Public License for more details.                                                                                                    
 *                                                                                                                                             
 * You should have received a copy of the GNU General Public License                                                                           
 * along with pigrade; see the file COPYING.  If not, write to the                                                                               
 * Free Software Foundation, Inc., 59 Temple Place - Suite 330,                                                                                
 * Boston, MA 02111-1307, USA.                                                                                                                 
 */

#include <sys/epoll.h>
#include <sys/socket.h>
#include <sys/ioctl.h>
#include <linux/can.h>
#include <net/if.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <netdb.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include <unistd.h>
#include <signal.h>
#include <array.h>
#include <assert.h>
#include <config.h>
#include <math.h>
//#include <conio.h> 
//#include <curses.h>
//#include <ncurses.h>
#include <kbhit.h>
#include <CAN_MQHP.h>
//#include <progress.h>
 #include "progressbar.h"
 #include "statusbar.h"
/* from extra part of simple ftp server's common.h */
#define _GNU_SOURCE
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <pwd.h>
#include <time.h>
#include <dirent.h>

/* from extra part of log2asc header files */
#include <time.h>
#include <libgen.h>
#include <sys/time.h>


#define FLAG_DAEMON 1
#define FLAG_LISTEN 2

#define __max(_x, _y)		({ typeof (_x) _a = (_x); typeof (_y) _b = (_y); _a > _b ? _a : _b; })
#define __min(_x, _y)		({ typeof (_x) _a = (_x); typeof (_y) _b = (_y); _a < _b ? _a : _b; })

#define DEBUG           1
#define __packed             __attribute__ ((__packed__))
//__attribute__ ((packed));

typedef struct __packed {
	uint8_t cmd;
	uint8_t nodeid;
	//uint8_t content[0];
	union __packed{
		struct __packed{ 
			uint8_t pack_hsty; //package history -- package nr to keep in fifo.//equal to group size.
			uint32_t pack_total_nbr;//must be multiple of pack_hsty.
			uint8_t __pad;
		};
		struct __packed{
			uint16_t pack_size; //contain the preheader and the realdata.
			uint32_t realdata_total_len; //unit: byte
		};
		struct __packed{
			uint8_t img_type[6]; //"pico" or "panel"
		};
		
	};
} hostmsg;

static unsigned short CRC16 ( unsigned char *puchMsg, unsigned short usDataLen );

static void hexdump(const void *_ptr, size_t len)
{
	const unsigned char *ptr = _ptr;
	unsigned int i;

	printf("hexdump ptr %p, len %zd\n", ptr, len);
	
	while (len > 0) {
		printf("%p: ", ptr);
		for (i=0; i < __min(len, 16u); i++) {
			printf("%02x ", *ptr);
			ptr++;
		}
		printf("\n");
		if (len < 16)
			break;
		len -= 16;
	}
}

// struct conn {
// 	int fd;
// 	struct sockaddr_in6 addr;

// 	union {
// 		struct can_frame frame[CONFIG_BUFFER_FRAMES];
// 		unsigned char raw[CONFIG_BUFFER_FRAMES * sizeof(struct can_frame)];
// 	} __attribute__((packed)) data;
// 	size_t dlen;
// };


/**
 * struct can_frame - basic CAN frame structure
 * @can_id:  CAN ID of the frame and CAN_*_FLAG flags, see canid_t definition
 * @can_dlc: frame payload length in byte (0 .. 8) aka data length code
 *           N.B. the DLC field from ISO 11898-1 Chapter 8.4.2.3 has a 1:1
 *           mapping of the 'data length code' to the real payload length
 * @__pad:   padding
 * @__res0:  reserved / padding
 * @__res1:  reserved / padding
 * @data:    CAN frame payload (up to 8 byte)
 */
#if 0

/*
 * Controller Area Network Error Message Frame Mask structure
 *
 * bit 0-28	: error class mask (see include/uapi/linux/can/error.h)
 * bit 29-31	: set to zero
 */
typedef __u32 can_err_mask_t;

/* CAN payload length and DLC definitions according to ISO 11898-1 */
#define CAN_MAX_DLC 8
#define CAN_MAX_DLEN 8

/* CAN FD payload length and DLC definitions according to ISO 11898-7 */
#define CANFD_MAX_DLC 15
#define CANFD_MAX_DLEN 64


/*
 * Controller Area Network Identifier structure
 *
 * bit 0-28	: CAN identifier (11/29 bit)
 * bit 29	: error message frame flag (0 = data frame, 1 = error message)
 * bit 30	: remote transmission request flag (1 = rtr frame)
 * bit 31	: frame format flag (0 = standard 11 bit, 1 = extended 29 bit)
 */
typedef __u32 canid_t;

#define CAN_SFF_ID_BITS		11
#define CAN_EFF_ID_BITS		29

struct can_frame {
	canid_t can_id;  /* 32 bit CAN_ID + EFF/RTR/ERR flags */
	__u8    can_dlc; /* frame payload length in byte (0 .. CAN_MAX_DLEN) */
	__u8    __pad;   /* padding */
	__u8    __res0;  /* reserved / padding */
	__u8    __res1;  /* reserved / padding */
	__u8    data[CAN_MAX_DLEN] __attribute__((aligned(8)));
};
#endif

struct conn {
	int fd;
	struct sockaddr_in addr;

	union {
		struct can_frame frame[CONFIG_BUFFER_FRAMES];
		unsigned char raw[CONFIG_BUFFER_FRAMES * sizeof(struct can_frame)];
	} __attribute__((packed)) data;
	size_t dlen;
};


struct can_iface {
	int fd;
	struct sockaddr_can addr;
};

static array_t *conns;
static array_t *ifaces;
static int run;


#define USE_TCP6_SOCKET 0
#if USE_TCP6_SOCKET
static int in6connect(const char *host, unsigned short port)
{
	struct addrinfo hints, *res, *p;
	char portstr[6];
	int ret_val, err;

	ret_val = -EHOSTUNREACH;
	memset(&hints, 0, sizeof(hints));
	hints.ai_family = AF_UNSPEC;
	hints.ai_socktype = SOCK_STREAM;
	snprintf(portstr, sizeof(portstr), "%hu", port);

	if((err = getaddrinfo(host, portstr, &hints, &res)) != 0) {
		fprintf(stderr, "getaddrinfo: %s\n", gai_strerror(err));
		ret_val = -ENOENT;
	} else {
		for(p = res; p; p = p->ai_next) {
			int fd;

			if((fd = socket(p->ai_family, p->ai_socktype, p->ai_protocol)) < 0) {
				ret_val = -errno;
				perror("socket");
				continue;
			}

			if((err = connect(fd, p->ai_addr, p->ai_addrlen)) < 0) {
				ret_val = -errno;
				perror("connect");
				close(fd);
			} else {
				ret_val = fd;
				break;
			}
		}
		
		freeaddrinfo(res);			
	}
	
	return(ret_val);
}

static int in6listen(unsigned short port)
{
	struct sockaddr_in6 addr;
	int err;
	int fd;

	if((fd = socket(PF_INET6, SOCK_STREAM, 0)) < 0) {
		err = errno;
		perror("socket");
		return(-err);
	}

	memset(&addr, 0, sizeof(addr));
	addr.sin6_family = AF_INET6;
	addr.sin6_port = htons(port);
	addr.sin6_addr = in6addr_any;
	err = 1;
	
	if(setsockopt(fd, SOL_SOCKET, SO_REUSEADDR, &err, sizeof(err)) < 0) {
		perror("setsockopt");
	}

	if(bind(fd, (struct sockaddr*)&addr, sizeof(addr)) < 0) {
		err = errno;
		perror("bind");
		close(fd);
		return(-err);
	}

	if(listen(fd, CONFIG_INET_BACKLOG) < 0) {
		err = errno;
		perror("listen");
		close(fd);
		return(-err);
	}

	return(fd);
}
#endif

/* refs: https://blog.csdn.net/qq_41822235/article/details/80789361 */

int getSign(unsigned num)     
{
    int sign = num & (1<<31);
    return sign == 0 ? 1 : -1; 
}
 
int getExp(unsigned num)      
{
    int exp = 0;
    for(int i = 23; i < 31; ++i)
        exp |= (num & (1<<i));
    exp = (exp>>23) - 127;
    return exp;
}
 
int float2int(float ft) 
{
    unsigned num;
    memcpy(&num, &ft, sizeof(float)); 
 
    int exp = getExp(num); 
    if(exp < 0) 
    {
        return 0;
    }
    else
    {
        int res = num & ((1<<23)-1);  
        res |= 1<<23;   
        res >>= (23-exp);
        return res*getSign(num);
    }
}
/* time control relations */
/*108   80   30   10*/
#define STD_FRAME_FULL_BIT_NBR 		108  
#define TARGET_CAN_RATE_KHZ    ((float)(1000))
#define TARGET_CAN_RATE_HZ     ((float)(TARGET_CAN_RATE_KHZ) * 1000)
#define REDUNDANT_TIME         120/* micro second */ //4,8,30(reduce some errs), 60,
//#define STD_FRAME_INTER_TIME   1
#define STD_FRAME_INTER_TIME   (float2int(((float)pow(10,6) / TARGET_CAN_RATE_HZ * (float)STD_FRAME_FULL_BIT_NBR)) + REDUNDANT_TIME)
#define STD_FRAME_INTER_TIME2   (float2int(((float)pow(10,6) / TARGET_CAN_RATE_HZ * (float)STD_FRAME_FULL_BIT_NBR)) + REDUNDANT_TIME)
// additional 2us for reduce alias.
//  t标准 = 1 / 目标速率 *10^6 * 108 


/* fill can_frame with specific data, and send send it to out_fd */
size_t send_with_canfrm(int out_fd, char* srcdata, size_t num, canid_t canid)
{
	int sent,sendlen,left;
    struct can_frame frm;
	uint64_t internal,time1,time2;
	struct timeval tv;
    //numSent = write(out_fd, data, num);
	internal = STD_FRAME_INTER_TIME;

    left = num;
    while(left){
		gettimeofday(&tv,NULL);
		time1 = (uint64_t)tv.tv_sec*1000000 + (uint64_t)tv.tv_usec;
		//printf("time1 %llu\n",time1);

        sendlen = left > 8 ? 8 : left;
        frm.can_id = canid;
        frm.can_dlc = sendlen;
        memcpy(frm.data, srcdata + num - left, sendlen);
        sent = write(out_fd, &frm, sizeof(frm));
        if(sent == -1) return -1;
        if(sent == 0) return -1;
        if(sent == sizeof(frm)){
            left -= sendlen;
        }else{//partial send failed.
            printf("failed: partial data sent\n");
            return -1;
        }

		/* wait for time slice expired */
		do{
			gettimeofday(&tv,NULL);
			time2 = (uint64_t)tv.tv_sec*1000000 + (uint64_t)tv.tv_usec;
		}while( time2 - time1 < internal );
		//printf("time2 %llu\n",time2);

    }
    if (left != 0){
		printf("send_with_canfrm failed: left != 0\n");
		exit_with_kbclose();
	}
	
    return num;
}

//#include "common.h"
#define BUF_SIZE 8192
static ssize_t sendfile(int out_fd, int in_fd, off_t * offset, int left )
{
    off_t orig;
    char buf[BUF_SIZE];
    int toRead, numRead, numSent, totSent;

    if (offset != NULL) {
        /* Save current file offset and set offset to value in '*offset' */
        orig = lseek(in_fd, 0, SEEK_CUR);
        if (orig == -1)
            return -1;
        if (lseek(in_fd, *offset, SEEK_SET) == -1)
            return -1;
    }

    totSent = 0;
    while (left > 0) {
        toRead = left<BUF_SIZE ? left : BUF_SIZE;

        numRead = read(in_fd, buf, toRead);
        if (numRead == -1)
            return -1;
        if (numRead == 0)
            break;                      /* EOF */

        numSent = send_with_canfrm(out_fd, buf, numRead, TOPIC_HOST_IMG_STREAM);
        if (numSent == -1)
            return -1;
        if (numSent == 0) {               /* Should never happen */
            perror("sendfile: send_with_canfrm() transferred 0 bytes");
            exit(-1);
        }

        left -= numSent;
        totSent += numSent;
    }

    if (offset != NULL) {
        /* Return updated file offset in '*offset', and reset the file offset
           to the value it had when we were called. */
        *offset = lseek(in_fd, 0, SEEK_CUR);
        if (*offset == -1)
            return -1;
        if (lseek(in_fd, orig, SEEK_SET) == -1)
            return -1;
    }
	printf("Send with ORIGINAL format CAN-ID: %04X \n",TOPIC_HOST_IMG_STREAM);
	printf("Transmit finished | total sent: %d bytes (without CAN struct wrapper)\n", totSent);
    return totSent;
}

/* send a file using pitech's MQHP protocol */
/* 
	typedef struct{
		uint32_t start_sig;
		uint32_t pack_total_nbr; //must be multiple of pack_hsty.
		uint32_t realdata_total_len; //unit: byte
		uint32_t pack_size;
		uint32_t pack_hsty; //package history - package nr to keep in fifo.
		uint8_t checkbuff[CHECK_BUFF_SIZE];
		uint8_t xor_result[CHECK_BUFF_SIZE];
		uint8_t img_type[8];
		uint8_t abort;
	} FEC_t;
	FEC_t fec;
*/
/*

		fec.pack_hsty = msg->pack_hsty; //package history -- package nr to keep in fifo.//equal to group size.
		fec.pack_total_nbr = msg->pack_total_nbr; //must be multiple of pack_hsty.
		break;
		case CMD_CONFIG_IMG_INFO2:
		fec.pack_size = msg->pack_size; //contain the preheader and the realdata.
		fec.realdata_total_len = msg->realdata_total_len; //unit: byte
		break;


 */


/* NOTE: packet size contain the whole FEC_packet_t size.*/
/* example: a packet size 1030 means it contain a preheader partial and a realdata partical. */


typedef struct __packed {
	uint16_t marker;//always be character "aa55"
	uint16_t serial_nbr; //start with '1', max to 64MB image. // 65536*1024/1024/1024
	uint16_t crc;
	//uint8_t data[0];

} FEC_packet_t;

#define MQ_PACK_DATA_SIZE        1024
#define MQ_PACK_SIZE             (MQ_PACK_DATA_SIZE + sizeof(FEC_packet_t))
#define MQ_HEADER_LEN            sizeof(FEC_packet_t) 
#define MQ_GROUP_SIZE            5 //alias of packet history
#define MQ_DIRTY_MARK            0x00
#define MQ_FILE_MAX_LEN          (64*1024*1024 - 256) //64M BYTES
//fixme: time period control


uint8_t xor_buff [MQ_PACK_SIZE+256] = { 0 };
void msg_xor( uint8_t *buff1, uint8_t *buff2, uint8_t *target, uint32_t len )
{
	for (size_t i = 0; i < len; i++){
		target[i] = buff1[i] ^ buff2[i];
	}
}

int calc_total_pkt_nbr(int len)
{
	int minpktnbr,remain,integer,totpktnbr,need_add_pkt;

	//calc the minimium pkt nbr to be send. 
	minpktnbr = (len + MQ_PACK_DATA_SIZE - 1) / MQ_PACK_DATA_SIZE;
	//calc the actually total pktnbr to be send...must be a nbr of multiple MQ_GROUP_SIZE
	//redundant_pkt = ( minpktnbr + (MQ_GROUP_SIZE - 2) ) / (MQ_GROUP_SIZE - 1);
	integer = minpktnbr / (MQ_GROUP_SIZE - 1);
	remain = minpktnbr - ( integer * (MQ_GROUP_SIZE - 1) );
	//additional redundant and dummy packet to satisfy condition: multiple of MQ_GROUP_SIZE
	need_add_pkt = integer + MQ_GROUP_SIZE - remain;
	totpktnbr = minpktnbr + need_add_pkt;

	return totpktnbr;
}

void exit_with_kbclose( void )
{
	close_keyboard();
	exit(-1);  /* EOF */ //should never read this 
}


int hashsum_cmdline(char* path, char* out_hash)
{
    FILE *fstream = NULL;
    int found = -1;
 
    memset(out_hash, 0, 96);  
 
    //?ping?? 
    //sprintf(buffer, "ping -c 1 %s", ip);
	sprintf(out_hash, "shasum %s", path);
    if (NULL == (fstream = popen(out_hash,"r"))){     
        return -1;      
    }   
 
    //??????
    while (NULL != fgets(out_hash, 96, fstream)) {  
        //LOG_INFO("%s", buffer);
        //?????????????
		break;
		found = 0;
        // if (strstr(buffer, "bytes from") != NULL)
        // {
        //     //???
        //     found = 0;
        //     break;
        // }
    }
 
    pclose(fstream);    

    return found;     
}

static ssize_t sendfileuseMQHP(char *img_type, int out_fd, int in_fd, off_t * offset, int len )
{
    off_t orig;
    unsigned char buf[MQ_PACK_SIZE + 256];
    int toRead, numRead, numSent, totSent = 0;
	int left = len;
	int sril_nbr = 0;
	int pkgcnt = 0;//sril cnt in a group 
	FEC_packet_t* packet = (FEC_packet_t*)buf;
	int totpktnbr,totpktnbr_const;
	int need_add;
	hostmsg msg;
	size_t ret;
	struct timeval tv1,tv2,t_begin,t_end;
	progressbar *bar;
	char hashvalue[96];

    if (offset != NULL) {
        /* Save current file offset and set offset to value in '*offset' */
        orig = lseek(in_fd, 0, SEEK_CUR);
        if (orig == -1)
            return -1;
        if (lseek(in_fd, *offset, SEEK_SET) == -1)
            return -1;
    }
	/* check the length of file prevent exceed 64M BYTES */
	if (len > MQ_FILE_MAX_LEN){
		printf("Error: input file exceed max file length !\n");
		exit(-1);
	}
	
	//printf("hostmsg: %d !\n",sizeof(hostmsg));
	/* starting send cmds of image config info  */
	msg.cmd = CMD_CONFIG_IMG_INFO1;
	msg.nodeid = NODE_ID_BROADCAST;
	msg.pack_hsty = MQ_GROUP_SIZE; //package history -- package nr to keep in fifo.//equal to group size.
	msg.pack_total_nbr = calc_total_pkt_nbr( len ); //must be multiple of pack_hsty.
	msg.__pad = MQ_DIRTY_MARK;
	ret = send_with_canfrm( out_fd, &msg, sizeof(hostmsg), TOPIC_HOST_GLOBAL_CMD);
	if(ret != sizeof(hostmsg)){
		printf("Error: CONFIG_IMG_INFO1 cannot send !\n");
		exit(-1);
	}
	usleep( 100*1000 ); 

	msg.cmd = CMD_CONFIG_IMG_INFO2;
	msg.nodeid = NODE_ID_BROADCAST;
	msg.pack_size = MQ_PACK_SIZE; //contain the preheader and the realdata.
	msg.realdata_total_len = len; //unit: byte
	ret = send_with_canfrm( out_fd, &msg, sizeof(hostmsg), TOPIC_HOST_GLOBAL_CMD);
	if(ret != sizeof(hostmsg)){
		printf("Error: CONFIG_IMG_INFO2 cannot send !\n");
		exit(-1);
	}
	usleep( 100*1000 ); 

	msg.cmd = CMD_START_SEND_IMG;/* config the transision img type */
	memcpy(msg.img_type, img_type, 6 );//"pico" or "panel"
	ret = send_with_canfrm( out_fd, &msg, sizeof(hostmsg), TOPIC_HOST_GLOBAL_CMD);
	if(ret != sizeof(hostmsg)){
		printf("Error: START_SEND_IMG cannot send !\n");
		exit(-1);
	}
	printf("LOG: send START_SEND_IMG request !\n");
	/* wait at least 500ms for panel prepare receive packet */
	usleep( 2*1000*1000 ); 
	/* Preprocess */
	totpktnbr = calc_total_pkt_nbr( len );
	totpktnbr_const = totpktnbr;
	printf("LOG: to be send [%d] packet  | file size [%d] bytes | grpnbr [%d]\n",totpktnbr,len, totpktnbr/MQ_GROUP_SIZE );
    totSent = 0;
	sril_nbr = 0;
	left = len;
	//clear xor_buff 
	memset(xor_buff, MQ_DIRTY_MARK, sizeof(xor_buff));
	init_keyboard();
	/* progress strip init */
	bar = progressbar_new_with_format("Progress", totpktnbr, "|#|");

	gettimeofday(&t_begin,NULL);

	/* every loop send a packet  */
	/* time control: send speed rate must < 1M/s( can bus max speed ) == 1K/s */
    while (totpktnbr--) {
		sril_nbr++;
		pkgcnt = (pkgcnt >= MQ_GROUP_SIZE ? 1 : pkgcnt+1);

		/* prepare the packet to be send */

		//first fill data part : numRead indicate the nbr read from the input file in a packet.
		if(pkgcnt != MQ_GROUP_SIZE){
			if(left){			
				toRead = left < MQ_PACK_DATA_SIZE ? left : MQ_PACK_DATA_SIZE;
				numRead = read(in_fd, buf + MQ_HEADER_LEN, toRead);	
				
				if (numRead == -1){
					printf("warning: numRead == -1.\n");
					exit_with_kbclose();
				}else if (numRead == 0){
					printf("warning: numRead == 0 (EOF).\n");
					exit_with_kbclose();  /* EOF */ //should never read this 
				}else if (numRead != toRead){
					printf("warning: input file numRead != toRead.\n");
					exit_with_kbclose();
				}else{//equal is ok
					left -= numRead;
					need_add = MQ_PACK_DATA_SIZE - numRead;
				}
			}else{
				need_add = MQ_PACK_DATA_SIZE;
			}

			if(need_add){ //fill the dummy data into empty part of the packet.
				memset( buf + MQ_PACK_DATA_SIZE - need_add, MQ_DIRTY_MARK, need_add );
			}

			//xor for FEC	
			msg_xor( xor_buff, buf + MQ_HEADER_LEN, xor_buff, MQ_PACK_DATA_SIZE );

		}else{// is the last pkt in this group.
			//fill data part 
			memcpy( buf + MQ_HEADER_LEN, xor_buff, MQ_PACK_DATA_SIZE );
			//clear xor_buff 
			memset(xor_buff, MQ_DIRTY_MARK, MQ_PACK_DATA_SIZE);
		}

		//next fill header part
		packet->marker = 0xaa55;
		packet->serial_nbr = sril_nbr;
		packet->crc = CRC16( buf + MQ_HEADER_LEN, MQ_PACK_DATA_SIZE);

		
		/* then sent a packet! */
        numSent = send_with_canfrm(out_fd, buf, MQ_PACK_SIZE, TOPIC_HOST_IMG_STREAM);
        if (numSent == -1){
			exit_with_kbclose();
		}else if (numSent == 0) {               /* Should never happen */
            perror("sendfile: send_with_canfrm() transferred 0 bytes");
            exit_with_kbclose();
        }else{ //send success.
			totSent += numSent;
		}
        	
		progressbar_inc(bar);
		//progress_show(&bar, (float)sril_nbr);


		/* exam if the user press the key 's'top */
		if(kbhit()){
			if(readch() == 's'){
				msg.cmd = CMD_STOP_SEND_IMG;/* config the transision img type */
				msg.nodeid = NODE_ID_BROADCAST;
				memcpy(msg.img_type, "123456", 6 );//"123456"
				ret = send_with_canfrm( out_fd, &msg, sizeof(hostmsg), TOPIC_HOST_GLOBAL_CMD);
				if(ret != sizeof(hostmsg)){
					printf("Error: CMD_STOP_SEND_IMG cannot send !\n");
					exit_with_kbclose();
				}
				printf("warning: user stop transmitting.\n");
				exit_with_kbclose();
			}
		}

    }

    if (offset != NULL) {
        /* Return updated file offset in '*offset', and reset the file offset
           to the value it had when we were called. */
        *offset = lseek(in_fd, 0, SEEK_CUR);
        if (*offset == -1)
            return -1;
        if (lseek(in_fd, orig, SEEK_SET) == -1)
            return -1;
    }
	//end handle
	progressbar_finish(bar);
	close_keyboard();
	gettimeofday(&t_end,NULL);
	printf("Elapse: [ %.4lf ] seconds\n",((double)(t_end.tv_sec*1000000 + t_end.tv_usec) - (double)(t_begin.tv_sec*1000000 + t_begin.tv_usec))/1000000);
	printf("Send with CAN-MQHP protocl CAN-ID ==> %04X \n",TOPIC_HOST_IMG_STREAM);
	printf("total sent: %d bytes ( with CAN-MQHP wrapper | without CANframe wrapper )\n", totSent);
	
    return len;
}


static int in4connect(const char *host, unsigned short port)
{
	struct addrinfo hints, *res, *p;
	char portstr[6];
	int ret_val, err;

	ret_val = -EHOSTUNREACH;
	memset(&hints, 0, sizeof(hints));
	hints.ai_family = AF_UNSPEC;
	hints.ai_socktype = SOCK_STREAM;
	snprintf(portstr, sizeof(portstr), "%hu", port);
	printf( "remote ip: %s | remote port: %s\n", host, portstr );
	if((err = getaddrinfo(host, portstr, &hints, &res)) != 0) {
		fprintf(stderr, "getaddrinfo: %s\n", gai_strerror(err));
		ret_val = -ENOENT;
	} else {
		for(p = res; p; p = p->ai_next) {
			int fd;

			if((fd = socket(p->ai_family, p->ai_socktype, p->ai_protocol)) < 0) {
				ret_val = -errno;
				perror("socket");
				continue;
			}
			// ?????
			//int nRecvBuf=32*1024;//???32K
			//setsockopt(fd, SOL_SOCKET, SO_RCVBUF,(const char*)&nRecvBuf,sizeof(int));
			// ?????
			//int nSendBuf = 1*1024*1024;//???32K
			int nSendBuf = 32*1024;//???32K
			setsockopt(fd, SOL_SOCKET, SO_SNDBUF,(const char*)&nSendBuf,sizeof(int));

			if((err = connect(fd, p->ai_addr, p->ai_addrlen)) < 0) {
				ret_val = -errno;
				perror("connect");
				close(fd);
			} else {
				ret_val = fd;
				break;
			}
		}
		
		freeaddrinfo(res);			
	}
	
	return(ret_val);
}

static int in4listen(unsigned short port)
{
	struct sockaddr_in addr;
	int err;
	int fd;

	if((fd = socket(PF_INET, SOCK_STREAM, 0)) < 0) {
		err = errno;
		perror("socket");
		return(-err);
	}

	memset(&addr, 0, sizeof(addr));
	addr.sin_family = AF_INET;
	addr.sin_addr.s_addr = htonl(INADDR_ANY);
	//addr.sin_addr.s_addr = inet_addr("192.168.101.164");
	addr.sin_port = htons(port);
	//sin_addr.s_addr=inet_addr("192.168.1.1");
	err = 1;
	
	if(setsockopt(fd, SOL_SOCKET, SO_REUSEADDR, &err, sizeof(err)) < 0) {
		perror("setsockopt");
	}

	//	?????
	int nRecvBuf= 32*1024;//???32K
	setsockopt(fd, SOL_SOCKET, SO_RCVBUF,(const char*)&nRecvBuf,sizeof(int));

	if(bind(fd, (struct sockaddr*)&addr, sizeof(addr)) < 0) {
		err = errno;
		perror("bind");
		close(fd);
		return(-err);
	}

	if(listen(fd, CONFIG_INET_BACKLOG) < 0) {
		err = errno;
		perror("listen");
		close(fd);
		return(-err);
	}

	return(fd);
}


static int cansock(void)
{
	struct sockaddr_can addr;
	struct ifreq ifr;
	int fd;
	int res;
	int e;

	if(!(ifaces = array_alloc())) {
		return(-1);
	}

	if((fd = socket(PF_CAN, SOCK_RAW, CAN_RAW)) < 0) {
		e = errno;
		perror("socket");
		errno = e;
		return(-1);
	}

	memset(&addr, 0, sizeof(addr));

	if(bind(fd, (struct sockaddr*)&addr, sizeof(addr)) < 0) {
		e = errno;
		perror("bind");
		close(fd);
		free(ifaces);
		ifaces = NULL;
		errno = e;
		return(-1);
	}
	
	memset(&ifr, 0, sizeof(ifr));
	ifr.ifr_ifindex = 1;
	
	do {
		if((res = ioctl(fd, SIOCGIFNAME, &ifr)) >= 0) {
			if(strstr(ifr.ifr_name, "can0") != NULL) {
				struct can_iface *iface;

				if((iface = malloc(sizeof(*iface)))) {
					memset(iface, 0, sizeof(*iface));
					
					iface->fd = fd;
					iface->addr.can_ifindex = ifr.ifr_ifindex;
					iface->addr.can_family = AF_CAN;

					if(array_insert(ifaces, iface) < 0) {
						free(iface);
					}
				}
				printf("found 'can0'\n");
			}
		}
		ifr.ifr_ifindex++;
	} while(res >= 0);

	return(fd);
}

static uint32_t cnt_ENOBUFS = 0;
static void broadcast_can(struct can_frame *frm)
{		
	uint64_t internal,time1,time2;
	struct timeval tv;

	internal = STD_FRAME_INTER_TIME2;
	gettimeofday(&tv,NULL);
	time1 = (uint64_t)tv.tv_sec*1000000 + (uint64_t)tv.tv_usec;
	ARRAY_FOREACH(ifaces, struct can_iface, iface, {
			if(sendto(iface->fd, frm, sizeof(*frm), 0, (struct sockaddr*)&(iface->addr), sizeof(iface->addr)) < 0) {
				//perror("CAN-sendto");
				if(errno == ENOBUFS){
					cnt_ENOBUFS++;
					if(cnt_ENOBUFS == 1){
						printf("cnt_ENOBUFS: %lu \n",cnt_ENOBUFS);
						printf("send speed is too fast -- missing a can frame\n",cnt_ENOBUFS);
						printf("FAILURE: transmit failed -- plz redo\n",cnt_ENOBUFS);
					}else if(cnt_ENOBUFS == 10){
						printf("cnt_ENOBUFS: %lu \n",cnt_ENOBUFS);
					}else if(cnt_ENOBUFS == 100){
						printf("cnt_ENOBUFS: %lu \n",cnt_ENOBUFS);
					}else if(cnt_ENOBUFS == 1000){
						printf("cnt_ENOBUFS: %lu \n",cnt_ENOBUFS);
					}else if(cnt_ENOBUFS == 10000){
						printf("cnt_ENOBUFS: %lu \n",cnt_ENOBUFS);
					}else if(cnt_ENOBUFS == 100000){
						printf("cnt_ENOBUFS: %lu \n",cnt_ENOBUFS);
					}else if(cnt_ENOBUFS == 1000000){
						printf("cnt_ENOBUFS: %lu \n",cnt_ENOBUFS);
					}else if(cnt_ENOBUFS == 10000000){
						printf("cnt_ENOBUFS: %lu \n",cnt_ENOBUFS);
					}else{
						;
					}
					
				}
				
			}
		});
	
	/* wait for time slice expired */
	do{
		gettimeofday(&tv,NULL);
		time2 = (uint64_t)tv.tv_sec*1000000 + (uint64_t)tv.tv_usec;
	}while( time2 - time1 < internal );
	return;
}

static void broadcast_net(struct can_frame *frm)
{
	ARRAY_FOREACH(conns, struct conn, con, {
			if(send(con->fd, frm, sizeof(*frm), 0) < 0) {
				perror("send");
			}
		});
	
	return;
}

static void broadcast_net2(struct can_frame *frm, struct conn *src)
{
	ARRAY_FOREACH(conns, struct conn, con, {
			if(con->fd == src->fd) {
				continue;
			}

			if(send(con->fd, frm, sizeof(*frm), 0) < 0) {
				perror("send");
			}
		});
	
	return;
}

static void handle_signal(int sig)
{
	switch(sig) {
	case SIGINT:
	case SIGHUP:
	case SIGTERM:
	case SIGUSR1:
		run = 0;
	default:
		break;
	}
	
	return;
}

static void sigsetup(void)
{
	struct sigaction sa;

	memset(&sa, 0, sizeof(sa));
	sa.sa_handler = handle_signal;

	sigaction(SIGINT, &sa, NULL);
	sigaction(SIGHUP, &sa, NULL);
	sigaction(SIGTERM, &sa, NULL);
	sigaction(SIGUSR1, &sa, NULL);

	return;
}

int verbose = 0;
int main(int argc, char *argv[])
{
	struct epoll_event ev[CONFIG_EPOLL_INITSIZE];
	char *hostname;
	int port;
	int epfd;
	int netfd;
	int canfd;
	int ret_val;
	int flags;
	int err;
	//FILE *infile = stdin;
	char *filepath = "no-spcecify";
	char *img_type = "orig  ";
	

	port = CONFIG_INET_PORT;
	flags = ~FLAG_DAEMON | FLAG_LISTEN;
	hostname = NULL;
	ret_val = 1;
	run = 1;
	
	for(ret_val = 1; ret_val < argc; ret_val++) {
		if(strcmp(argv[ret_val], "--daemon") == 0 || strcmp(argv[ret_val], "-d") == 0) {
			flags |= FLAG_DAEMON;
		} else if(strcmp(argv[ret_val], "--connect") == 0 || strcmp(argv[ret_val], "-c") == 0) {
			if(++ret_val < argc) {
				flags &= ~FLAG_LISTEN;
				hostname = argv[ret_val];
			} else {
				fprintf(stderr, "Expected destination after --connect\n");
				return(1);
			}
		} else if(strcmp(argv[ret_val], "--port") == 0 || strcmp(argv[ret_val], "-p") == 0) {
			if(++ret_val < argc) {
				port = strtol(argv[ret_val], NULL, 10);

				if(port > (1 << 16) || errno == ERANGE) {
					fprintf(stderr, "Invalid port specified\n");
					return(1);
				}
			} else {
				fprintf(stderr, "Expected port number after --port\n");
				return(1);
			}
		} else if(strcmp(argv[ret_val], "--help") == 0 || strcmp(argv[ret_val], "-h") == 0) {
			printf("Welcome: %s \n"
					"\n"
				   "[INFO]: \n"
				   "Provide an IP-to-CAN file transition with CAN-MQHP protocol for PITECH device upgrade\n"
				   "By default, %s will not run as a daemon.\n"
				   "By default, the transion protocol is file original format wrapped with can_frame_t struct\n"
				   "The CAN-MQHP protocl will used at the option 'type' which is not orig type\n"
				   "The file should smaller than 64 Mbytes\n"
				   "It can listen for multi incoming connections on port %d.\n"
				   "\n"
				   "[OPTIONS]: \n"
				   "  -h, --help        display this help and exit\n"
				   "  -d, --daemon      run as a daemon (fork to the background)\n"
				   "  -c, --connect     connect to the host specified by the next argument\n"
				   "  -p, --port        use the port specified by the next argument\n"
				   "  -I, --input       indicate the file path to be transfer(default from stdin)\nnote: max to 64MB file\n"
				   "  -t, --type        specify the image type of the file( param: orig / pico / panel )\n"
				   "  -v, --verbose     display the details of TX/RX info\n",
				   argv[0], CONFIG_MY_NAME, CONFIG_INET_PORT);
			return(1);
		} else if(strcmp(argv[ret_val], "--input") == 0 || strcmp(argv[ret_val], "-I") == 0) {
			if(++ret_val < argc) {
				
				filepath =	argv[ret_val];
				// infile = fopen(filepath, "r");
				// if (!infile) {
				// 	perror("infile");
				// 	return 1;
				// }
			} else {
				fprintf(stderr, "Expected file path to be transfer after --input\n");
				return(1);
			}
		} else if(strcmp(argv[ret_val], "--type") == 0 || strcmp(argv[ret_val], "-t") == 0) {
			if(++ret_val < argc) {
				if (argv[ret_val][1] == 'a'){
					img_type = "panel "; //keep 6 bytes.
				}else if(argv[ret_val][1] == 'i'){
					img_type = "pico  "; //keep 6 bytes.
				}else{
					img_type = "orig  "; //keep 6 bytes.
				}	

			} else {
				fprintf(stderr, "Expected image type after --input\n");
				return(1);
			}
		} else if(strcmp(argv[ret_val], "--verbose") == 0 || strcmp(argv[ret_val], "-v") == 0) {
			verbose = 1;
		}
	}

	if(!(conns = array_alloc())) {
		perror("array_alloc");
		return(1);
	}

	ret_val = 1;

	if(flags & FLAG_DAEMON) {
		int pid;

		pid = fork();

		if(pid > 0) {
			return(0);
		} else if(pid < 0) {
			perror("fork");
			return(1);
		}
		
		setsid();
	}
	sigsetup();

	if((canfd = cansock()) < 0) {
		fprintf(stderr, "Failed to initialize CAN socket\n");
		return(1);
	}

	if((epfd = epoll_create(CONFIG_EPOLL_INITSIZE)) < 0) {
		perror("epoll_create");
		close(canfd);
		return(1);
	} else {
		ev[0].data.ptr = &canfd;
		ev[0].events = EPOLLIN;

		if(epoll_ctl(epfd, EPOLL_CTL_ADD, canfd, &ev[0]) < 0) {
			perror("epoll_ctl");
			close(epfd);
			close(canfd);
			return(1);
		}
	}
	
	if(flags & FLAG_LISTEN) { //runing as server mode for receive binary file and broadcast to CANbus.
		if((netfd = in4listen(port & 0xffff)) >= 0) {
			ev[0].data.ptr = &netfd;
			ev[0].events = EPOLLIN;

			if(epoll_ctl(epfd, EPOLL_CTL_ADD, netfd, &ev[0]) < 0) {
				perror("epoll_ctl");
				ret_val = -1;
			} else{
				while(run) {
					int n = epoll_wait(epfd, ev, CONFIG_EPOLL_INITSIZE, -1);

					while(--n >= 0) {
						struct conn *con;

						con = ev[n].data.ptr;

						/* If con->fd is netfd or canfd, don't use any other members of con!
						   In these cases, con really points to just an int, not a struct conn! */

						if(con->fd == netfd) {
							/* new TCP connection -> set up connection */
						
							struct epoll_event nev;
							struct conn *new_con;
							socklen_t addrlen;

							if(!(new_con = malloc(sizeof(*new_con)))) {
								perror("malloc");
							} else {
								new_con->fd = accept(con->fd, (struct sockaddr*)&(new_con->addr), &addrlen);
								
								char *address_str = inet_ntoa(new_con->addr.sin_addr);
								//char *inet_ntoa(struct in_addr);
								printf("a NEW connection from: %s \n\n",address_str);
								if(new_con->fd < 0) {
									free(new_con);
								} else {
									nev.data.ptr = new_con;
									nev.events = EPOLLIN;

									if(epoll_ctl(epfd, EPOLL_CTL_ADD, new_con->fd, &nev) < 0) {
										perror("epoll_ctl");
										close(new_con->fd);
										free(new_con);
									} else if((err = array_insert(conns, new_con)) < 0) {
										fprintf(stderr, "array_insert: %s\n", strerror(-err));
										close(new_con->fd);
										free(new_con);
									}									
								}
							}
						} else if(con->fd == canfd) {
							/* message from CAN bus -> broadcast to TCP clients */

							struct can_frame frm;
							int flen;

							if((flen = read(con->fd, &frm, sizeof(frm))) < 0) {
								perror("read");
							} else if(flen == sizeof(frm)) {
								if(verbose){
									printf("recv from CAN: ");
									hexdump(&frm, sizeof(frm));
									printf("\n");
								}
								broadcast_net(&frm);
							} else {
								fprintf(stderr, "flen = %d\n", flen);
							}
						} else {
							/* message from TCP client -> broadcast to TCP clients and CAN bus */

							size_t rsize;
							int rd;
						
							rsize = sizeof(con->data) - con->dlen;

							if(rsize > 0) {
								rd = recv(con->fd, con->data.raw + con->dlen, rsize, 0);

								if(rd > 0) {
									con->dlen += rd;
								}
							}

							/* broadcast buffered frames */
							if(con->dlen >= sizeof(struct can_frame)) {
								size_t new_dlen;
								int idx;

								idx = 0;
								new_dlen = con->dlen;
							
								/* send out the frames that were fully buffered */
							
								while(idx < CONFIG_BUFFER_FRAMES && new_dlen >= sizeof(struct can_frame)) {
									if(verbose){
										printf("recv from tcp: ");
										hexdump(&(con->data.frame[idx]), sizeof(struct can_frame));
										printf("\n");
									}
									broadcast_can(&(con->data.frame[idx]));
									broadcast_net2(&(con->data.frame[idx]), con);
									idx++;
									new_dlen -= sizeof(struct can_frame);
								}

								/* if we have a partial frame, move it to the front of the buffer */
								if(new_dlen > 0) {
									memcpy(con->data.raw, con->data.raw + (con->dlen - new_dlen), new_dlen);
								}
							
								con->dlen = new_dlen;
							}

							if(rd <= 0) {
								/* error (-1) or connection closed (0) */
								array_remove(conns, con);
								close(con->fd);
								free(con);
							}
						}
					}
				} /* while(run) */
			}
			close(netfd);
		}

	} else { /* runing as client mode for trasfer binary file.*/
		/* Client mode */
		if((netfd = in4connect(hostname, port & 0xffff)) < 0) {
			fprintf(stderr, "Unable to connect to %s:%d\n", hostname, port & 0xffff);
		}else{//connetion is OK.
			printf("Connected to %s:%d\n", hostname, port & 0xffff);
			int transfd;
			struct stat stat_buf;
			off_t offset = 0;
			int sent_total = 0;
			char cwd[96];
			char *ret;
			char hashsum[96];

			if(strcmp(filepath, "no-spcecify") == 0){
				printf("file path no spcecify, use default path.\n");
				ret = getcwd(cwd,sizeof(cwd));
				if(ret == NULL) printf("failed: cwd len exceed.\n");
				return (0);
				strncat(cwd,"file.bin",20);
				printf("default path: %s\n", cwd );
				filepath = cwd;
			}else{
				;//path already assigned
			}


			if(access(filepath,R_OK)==0 && (transfd = open(filepath,O_RDONLY))){
				fstat(transfd,&stat_buf);
				printf("Found file --> starting BINARY file data transfer.\n");
				/* do hash */
				hashsum_cmdline(filepath, hashsum);
				printf("SHA: %s", hashsum );

				if ( strcmp(img_type, "orig  ") == 0 ){
					sent_total = sendfile(netfd, transfd, &offset, stat_buf.st_size);
				}else{
					sent_total = sendfileuseMQHP(img_type, netfd, transfd, &offset, stat_buf.st_size);
				}
				if(sent_total){
					if(sent_total != stat_buf.st_size){
						perror("sendfile");
						printf("File send size not match.\n");
						exit(EXIT_SUCCESS);
					}else{
						printf("File send successful.\n");
					}
				}else{
					printf("Failed to read file.sent_total = 0.\n");
					exit(EXIT_SUCCESS);
				}
				
			}else{
				printf("File cannot found/open.\n");
				exit(EXIT_SUCCESS);
			}
			close(transfd);
			close(netfd);
			exit(EXIT_SUCCESS);
			return (0);
		}

	}

	close(epfd);
	close(canfd);
	
	return(ret_val);
}



/*
	CRC16 MODULE FORM MODBUS
 */
static unsigned char auchCRCHi[];
static char auchCRCLo[];

static unsigned short CRC16 ( unsigned char *puchMsg, unsigned short usDataLen ) 
/* The function returns the CRC as a unsigned short type */
//unsigned char *puchMsg ; /* message to calculate CRC upon */
//unsigned short usDataLen ; /* quantity of bytes in message */
{
unsigned char uchCRCHi = 0xFF ; /* high byte of CRC initialized */
unsigned char uchCRCLo = 0xFF ; /* low byte of CRC initialized */
unsigned uIndex ; /* will index into CRC lookup table */
	while (usDataLen--) /* pass through message buffer */
	{
		uIndex = uchCRCLo ^ *puchMsg++ ; /* calculate the CRC */
		uchCRCLo = uchCRCHi ^ auchCRCHi[uIndex] ;
		uchCRCHi = auchCRCLo[uIndex] ;
	}
return (uchCRCHi << 8 | uchCRCLo) ;
}

/* Table of CRC values for high–order byte */
static unsigned char auchCRCHi[] = {
0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81,
0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0,
0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01,
0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41,
0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81,
0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0,
0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01,
0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81,
0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0,
0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01,
0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81,
0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0,
0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01,
0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81,
0x40
} ;


/* Table of CRC values for low–order byte */
static char auchCRCLo[] = {
0x00, 0xC0, 0xC1, 0x01, 0xC3, 0x03, 0x02, 0xC2, 0xC6, 0x06, 0x07, 0xC7, 0x05, 0xC5, 0xC4,
0x04, 0xCC, 0x0C, 0x0D, 0xCD, 0x0F, 0xCF, 0xCE, 0x0E, 0x0A, 0xCA, 0xCB, 0x0B, 0xC9, 0x09,
0x08, 0xC8, 0xD8, 0x18, 0x19, 0xD9, 0x1B, 0xDB, 0xDA, 0x1A, 0x1E, 0xDE, 0xDF, 0x1F, 0xDD,
0x1D, 0x1C, 0xDC, 0x14, 0xD4, 0xD5, 0x15, 0xD7, 0x17, 0x16, 0xD6, 0xD2, 0x12, 0x13, 0xD3,
0x11, 0xD1, 0xD0, 0x10, 0xF0, 0x30, 0x31, 0xF1, 0x33, 0xF3, 0xF2, 0x32, 0x36, 0xF6, 0xF7,
0x37, 0xF5, 0x35, 0x34, 0xF4, 0x3C, 0xFC, 0xFD, 0x3D, 0xFF, 0x3F, 0x3E, 0xFE, 0xFA, 0x3A,
0x3B, 0xFB, 0x39, 0xF9, 0xF8, 0x38, 0x28, 0xE8, 0xE9, 0x29, 0xEB, 0x2B, 0x2A, 0xEA, 0xEE,
0x2E, 0x2F, 0xEF, 0x2D, 0xED, 0xEC, 0x2C, 0xE4, 0x24, 0x25, 0xE5, 0x27, 0xE7, 0xE6, 0x26,
0x22, 0xE2, 0xE3, 0x23, 0xE1, 0x21, 0x20, 0xE0, 0xA0, 0x60, 0x61, 0xA1, 0x63, 0xA3, 0xA2,
0x62, 0x66, 0xA6, 0xA7, 0x67, 0xA5, 0x65, 0x64, 0xA4, 0x6C, 0xAC, 0xAD, 0x6D, 0xAF, 0x6F,
0x6E, 0xAE, 0xAA, 0x6A, 0x6B, 0xAB, 0x69, 0xA9, 0xA8, 0x68, 0x78, 0xB8, 0xB9, 0x79, 0xBB,
0x7B, 0x7A, 0xBA, 0xBE, 0x7E, 0x7F, 0xBF, 0x7D, 0xBD, 0xBC, 0x7C, 0xB4, 0x74, 0x75, 0xB5,
0x77, 0xB7, 0xB6, 0x76, 0x72, 0xB2, 0xB3, 0x73, 0xB1, 0x71, 0x70, 0xB0, 0x50, 0x90, 0x91,
0x51, 0x93, 0x53, 0x52, 0x92, 0x96, 0x56, 0x57, 0x97, 0x55, 0x95, 0x94, 0x54, 0x9C, 0x5C,
0x5D, 0x9D, 0x5F, 0x9F, 0x9E, 0x5E, 0x5A, 0x9A, 0x9B, 0x5B, 0x99, 0x59, 0x58, 0x98, 0x88,
0x48, 0x49, 0x89, 0x4B, 0x8B, 0x8A, 0x4A, 0x4E, 0x8E, 0x8F, 0x4F, 0x8D, 0x4D, 0x4C, 0x8C,
0x44, 0x84, 0x85, 0x45, 0x87, 0x47, 0x46, 0x86, 0x82, 0x42, 0x43, 0x83, 0x41, 0x81, 0x80,
0x40
};





/* orginal part of client function -- gateway. */
#if 0 
		 else {
			ev[0].data.ptr = &netfd;
			ev[0].events = EPOLLIN;

			if(epoll_ctl(epfd, EPOLL_CTL_ADD, netfd, &ev[0]) < 0) {
				perror("epoll_ctl");
			} else {
				union {
					struct can_frame frame[CONFIG_BUFFER_FRAMES];
					unsigned char raw[CONFIG_BUFFER_FRAMES * sizeof(struct can_frame)];
				} buffer;
				size_t dlen;
				int n;

				dlen = 0;

				while(run) {
					n = epoll_wait(epfd, ev, CONFIG_EPOLL_INITSIZE, -1);//block until a event occurs.

					while(--n >= 0) {
						int fd = *((int*)ev[n].data.ptr);
						int len;
						
						if(fd == canfd) {//from can msg
							struct can_frame frm;

							len = read(fd, &frm, sizeof(frm));

							if(len == sizeof(frm)) {
								if(send(netfd, &frm, sizeof(frm), 0) < 0) {
									perror("send");
									close(netfd);
									run = 0;
								}
							}
						} else {//from net msg
							size_t rsize;
						
							rsize = sizeof(buffer) - dlen;

							if(rsize > 0) {
								len = recv(netfd, buffer.raw + dlen, rsize, 0);

								if(len > 0) {
									dlen += len;
								}
							}

							/* broadcast buffered frames */
							if(dlen >= sizeof(struct can_frame)) {
								size_t new_dlen;
								int idx;

								idx = 0;
								new_dlen = dlen;
							
								/* send out the frames that were fully buffered */
							
								while(idx < CONFIG_BUFFER_FRAMES && new_dlen >= sizeof(struct can_frame)) {
									broadcast_can(&(buffer.frame[idx]));
									idx++;
									new_dlen -= sizeof(struct can_frame);
								}

								/* if we have a partial frame, move it to the front of the buffer */
								if(new_dlen > 0) {
									memcpy(buffer.raw, buffer.raw + (dlen - new_dlen), new_dlen);
								}
							
								dlen = new_dlen;
							}

							if(len <= 0) {
								/* error (-1) or connection closed (0) */
								close(netfd);
								run = 0;
							}
						}
					}
				}
			}
		}

#endif