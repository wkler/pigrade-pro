

/*

"_sleep()" can be replaced by a number of Linux (UNIX) functions, depending on the time resolution. Below are some of these functions (reference the man pages).

sleep() - second resolution
usleep() - microsecond resolution
nanosleep() - nanosecond resolution


getch() is getchar() or use the readch() function I provide below.

kbhit() is not available on Linux (UNIX), it is a DOS function. Below is some code (functions) to replace the "kbhit" functionality (this code is from a book called "Beginning Linux Programming", from Wrox Press -- www.wrox.com -- you can download the Code Examples from the Book at this Site):

*/

#include "kbhit.h"
#include <termios.h>
#include <unistd.h>   // for read()

static struct termios initial_settings, new_settings;
static int peek_character = -1;

void init_keyboard()
{
    tcgetattr(0,&initial_settings);
    new_settings = initial_settings;
    new_settings.c_lflag &= ~ICANON;
    new_settings.c_lflag &= ~ECHO;
    new_settings.c_lflag &= ~ISIG;
    new_settings.c_cc[VMIN] = 1;
    new_settings.c_cc[VTIME] = 0;
    tcsetattr(0, TCSANOW, &new_settings);
}

void close_keyboard()
{
    tcsetattr(0, TCSANOW, &initial_settings);
}

int kbhit()
{
    unsigned char ch;
    int nread;

    if (peek_character != -1) return 1;
    new_settings.c_cc[VMIN]=0;
    tcsetattr(0, TCSANOW, &new_settings);
    nread = read(0,&ch,1);
    new_settings.c_cc[VMIN]=1;
    tcsetattr(0, TCSANOW, &new_settings);
    if(nread == 1)
    {
        peek_character = ch;
        return 1;
    }
    return 0;
}

int readch()
{
    char ch;

    if(peek_character != -1)
    {
        ch = peek_character;
        peek_character = -1;
        return ch;
    }
    read(0,&ch,1);
    return ch;
}


#if 0

#include <unistd.h> 
#include <stdio.h> 
#include <ctype.h> 

#include "kbhit.h" /* http://linux-sxs.org/programming/kbhit.html */

int main(){
  init_keyboard();
  char ch='x';
  while( ch != 'q' ){
    printf("looping\n");
    sleep(1);
    if( kbhit() ){
      printf("you hit");
      do{
        ch = readch();
        printf(" '%c'(%i)", isprint(ch)?ch:'?', (int)ch );
      }while( kbhit() );
      puts("");
    }
  }
  close_keyboard();
}

#endif