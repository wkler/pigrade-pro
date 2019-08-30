
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


#include "common.h"
#ifndef sendfile
#define BUF_SIZE 8192
ssize_t sendfile(int out_fd, int in_fd, off_t * offset, size_t count )
{
    off_t orig;
    char buf[BUF_SIZE];
    size_t toRead, numRead, numSent, totSent;

    if (offset != NULL) {
        /* Save current file offset and set offset to value in '*offset' */
        orig = lseek(in_fd, 0, SEEK_CUR);
        if (orig == -1)
            return -1;
        if (lseek(in_fd, *offset, SEEK_SET) == -1)
            return -1;
    }

    totSent = 0;
    while (count > 0) {
        toRead = count<BUF_SIZE ? count : BUF_SIZE;

        numRead = read(in_fd, buf, toRead);
        if (numRead == -1)
            return -1;
        if (numRead == 0)
            break;                      /* EOF */

        numSent = send_with_canfrm(out_fd, buf, numRead);
        if (numSent == -1)
            return -1;
        if (numSent == 0) {               /* Should never happen */
            perror("sendfile: send_with_canfrm() transferred 0 bytes");
            exit(-1);
        }

        count -= numSent;
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
    return totSent;
}
#endif



// #include <stdio.h>
// #include <sys/time.h>
// #include <time.h>
// int gettimeofday(struct timeval *tv, struct timezone *tz);
// int main(int argc,char * argv[])
// {
// 　　struct timeval tv;
// 　　while(1)
//     {
// 　　gettimeofday(&tv,0);
// 　　printf("time %u:%u\n",tv.tv_sec,tv.tv_usec);
// 　　sleep(2);
// 　　}
// 　　return 0;
// }


#define TOPIC_HOST_IMG_STREAM          (0x05FF)U
/* fill can_frame with specific data, and send send it to out_fd */
size_t send_with_canfrm(int out_fd, char* srcdata, size_t num)
{
    size_t i,left,sendlen,sent;
    struct can_frame frm;
    //numSent = write(out_fd, data, num);
    left = num;
    while(left){
        sendlen = left > 8 ? 8 : left;
        frm.can_id = TOPIC_HOST_IMG_STREAM;
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
    }
    
    return (num - left);
}



void transfile2conn(char *filepath, int connection)
{

  //if(fork()==0){
    int fd;
    struct stat stat_buf;
    off_t offset = 0;
    int sent_total = 0;

    if(access(filepath,R_OK)==0 && (fd = open(filepath,O_RDONLY))){
        fstat(fd,&stat_buf);
        printf("starting BINARY file data transfer.\n");
        //connection = accept_connection(state->sock_pasv);
        //close(state->sock_pasv);
        if(sent_total = sendfile(connection, fd, &offset, stat_buf.st_size)){
            if(sent_total != stat_buf.st_size){
                perror("sendfile");
                printf("File send size not match.\n");
                exit(EXIT_SUCCESS);
            }else{
                printf("File send OK.\n");
            }
        }else{
            printf("Failed to read file.sent_total = 0.\n");
            exit(EXIT_SUCCESS);
        }
    }else{
        printf("cannot find/open file.\n");
        exit(EXIT_SUCCESS);
    }

    close(fd);
    //close(connection);
    //exit(EXIT_SUCCESS);
  //}
    //close(state->sock_pasv);
}







/* monitor the file changing */

#if 0

#include <sys/epoll.h>  
#include <fcntl.h>  
#include <stdio.h>  
#include <errno.h>  
#include <stdlib.h>  
#include <sys/inotify.h>  
  
#define EVENT_SIZE  ( sizeof (struct inotify_event) )  
#define BUF_LEN     ( 1024 * ( EVENT_SIZE + 16 ) )  
  
int  
main (int argc, char *argv[])  
{  
  int i, epfd, nfds, fd;  
  int wd;  
  int length;  
  
  char buffer[BUF_LEN];  
  struct epoll_event ev, events[20];  
  epfd = epoll_create (256);  
  fd = inotify_init ();  
  wd = inotify_add_watch (fd, "/home/cc/tmp",  
                          IN_MODIFY | IN_CREATE | IN_DELETE);  
  
  ev.data.fd = fd;  
  ev.events = EPOLLIN | EPOLLET;  
  
  epoll_ctl (epfd, EPOLL_CTL_ADD, fd, &ev);  
  
  for (;;)  
    {  
      nfds = epoll_wait (epfd, events, 20, 500);  
  
      for (i = 0; i < nfds; ++i)  
        {  
          if (events[i].data.fd == fd)
            {  
              length = read (fd, buffer, BUF_LEN);  
  
              if (length < 0)  
                {  
                  perror ("read");  
                }  
  
              while (i < length)  
                {  
                  struct inotify_event *event =  
                    (struct inotify_event *) &buffer[i];  
                  if (event->len)  
                    {  
                      if (event->mask & IN_CREATE)  
                        {  
                          if (event->mask & IN_ISDIR)  
                            {  
                              printf ("The directory %s was created.\n",  
                                      event->name);  
                            }  
                          else  
                            {printf ("The file %s was created.\n",  
                                      event->name);  
                            }  
                        }  
                      else if (event->mask & IN_DELETE)  
                        {  
                          if (event->mask & IN_ISDIR)  
                            {  
                              printf ("The directory %s was deleted.\n",  
                                      event->name);  
                            }  
                          else  
                            {  
                              printf ("The file %s was deleted.\n",  
                                      event->name);  
                            }  
                        }  
                      else if (event->mask & IN_MODIFY)  
                        {  
                          if (event->mask & IN_ISDIR)  
                            {  
                              printf ("The directory %s was modified.\n",  
                                      event->name);  
                            }  
                          else  
                            {  
                              printf ("The file %s was modified.\n",  
                                      event->name);  
                            }  
                        }  
                    }  
                  i += EVENT_SIZE + event->len;  
                }  
          
            }  
  
  
        }  
  
    }  
  
  
  
  return 0;  
} 

#endif

/**
 * Send file specified in filename over data connection, sending
 * control message over control connection
 * Handles case of null or invalid filename
 */
void ftserve_retr(int sock_control, int sock_data, char* filename)
{	
	FILE* fd = NULL;
	char data[MAXSIZE];
	size_t num_read;							
		
	fd = fopen(filename, "r");
	
	if (!fd) {	
		// send error code (550 Requested action not taken)
		send_response(sock_control, 550);
		
	} else {	
		// send okay (150 File status okay)
		send_response(sock_control, 150);
	
		do {
			num_read = fread(data, 1, MAXSIZE, fd);

			if (num_read < 0) {
				printf("error in fread()\n");
			}

			// send block
			if (send(sock_data, data, num_read, 0) < 0)
				perror("error sending file\n");

		} while (num_read > 0);													
			
		// send message: 226: closing conn, file transfer successful
		send_response(sock_control, 226);

		fclose(fd);
	}
}







	// 	struct timeval{
	// long tv_sec; //秒
	// long tv_usec; //微秒
	// };
	// struct timezone
	// {
	// int tz_minuteswest; //和Greenwich 时间差了多少分钟
	// int tz_dsttime; //日光节约时间的状态
	// }; 
int gettimeofday (struct timeval * tv, struct timezone * tz)
	gettimeofday(&tv,NULL);

  
	struct timeval tv;
    gettimeofday(&tv,NULL);
    printf("second:%ld\n",tv.tv_sec);  //秒
    printf("millisecond:%ld\n",tv.tv_sec*1000 + tv.tv_usec/1000);  //毫秒
    printf("microsecond:%ld\n",tv.tv_sec*1000000 + tv.tv_usec);  //微秒



    1.unsigned int sleep(unsigned int seconds);  
  sleep()会使当前程序休眠seconds秒。如果sleep()没睡饱，它将会返回还需要补眠的时间，否则一般返回零。 
 
2.void usleep(unsigned long usec);  
 usleep与sleep()类同，不同之处在于休眠的时间单位为毫秒（10E-6秒）。 
 
3.int select(0,NULL,NULL,NULL,struct timeval *tv);  
 
  可以利用select实现sleep()的功能，它将不会等待任何事件发生。 
 
4.int nanosleep(struct timespec *req,struct timespec *rem);  
  nanosleep会沉睡req所指定的时间，若rem为non-null，而且没睡饱，将会把要补眠的时间放在rem上。

 ———————————————— 
版权声明：本文为CSDN博主「YongXMan」的原创文章，遵循CC 4.0 BY-SA版权协议，转载请附上原文出处链接及本声明。
原文链接：https://blog.csdn.net/geekcome/article/details/6554729

实际上用select是万能的，下面的是一个使用select的例子：

#include <time.h>
#include <sys/time.h>
 
void Sleep(int iSec,int iUsec)
{
       struct timeval tv;
      tv.tv_sec=iSec;
      tv.tv_usec=iUsec;
      select(0,NULL,NULL,NULL,&tv);
}
iSec 为延时秒数,Usec为延时微秒数.

注:
1秒=1000毫秒=1000000微秒=1000000000纳秒=1000000000000皮秒=1000000000000000飞秒
1s=1000ms=1000000us=1000000000ns=1000000000000ps=1000000000000000fs
 ———————————————— 
版权声明：本文为CSDN博主「YongXMan」的原创文章，遵循CC 4.0 BY-SA版权协议，转载请附上原文出处链接及本声明。
原文链接：https://blog.csdn.net/geekcome/article/details/6554729