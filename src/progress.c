/**
 * linux terminal progress bar (no thread safe).
 * 	@package progress.c
 *
 * @author chenxin <chenxin619315@gmail.com>
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "progress.h"

/**
 * initialize the progress bar.
 * @max = 0
 * @val = 0
 *
 * @param	style
 * @param	tip words.
 */
extern void progress_init( progress_t *bar, char *title, int max, int style )
{
    bar->chr = '#';
    bar->title = title;
    bar->style = style;
    bar->max = max;
    bar->offset = 100 / (float)max;
    bar->pro = (char *) malloc(max+1);
    if ( style == PROGRESS_BGC_STYLE )
	memset(bar->pro, 0x00, max+1);
    else {
	memset(bar->pro, 32, max);
	memset(bar->pro+max, 0x00, 1);
    }
}

extern void progress_show( progress_t *bar, float bit )
{
    int val = (int)(bit * bar->max);
    switch ( bar->style ) 
    {
    case PROGRESS_NUM_STYLE:
	    printf("\033[?25l\033[31m\033[1m%s%d%%\033[?25h\033[0m\r",
		bar->title, (int)(bar->offset * val));
        fflush(stdout);
        break;
    case PROGRESS_CHR_STYLE:
        memset(bar->pro, '#', val);
        printf("\033[?25l\033[31m\033[1m%s[%-s] %d%%\033[?25h\033[0m\r", 
            bar->title, bar->pro, (int)(bar->offset * val));
        fflush(stdout);
        break;
    case PROGRESS_BGC_STYLE:
        memset(bar->pro, 32, val);
        printf("\033[?25l\033[31m\033[1m%s\033[41m %d%% %s\033[?25h\033[0m\r", 
            bar->title, (int)(bar->offset * val), bar->pro);
        fflush(stdout);
        break;
     default:
        break;
    }
}

//destroy the the progress bar.
extern void progress_destroy(progress_t *bar)
{
    free(bar->pro);
}



/* test routine */

/**
 * program bar test program.
 *
 * @author chenxin <chenxin619315@gmail.com>
 */
/*
#include "progress.h"
#include <unistd.h>

int main(int argc, char *argv[] )
{
    progress_t bar;
    //progress_init(&bar, "", 50, PROGRESS_NUM_STYLE);
    progress_init(&bar, "", 50, PROGRESS_CHR_STYLE);
    //progress_init(&bar, "", 50, PROGRESS_BGC_STYLE);

    int i;
    for ( i = 0; i <= 50; i++ ) {
	progress_show(&bar, i/50.0f);
	sleep(1);
    }
    printf("\n+-Done\n");

    progress_destroy(&bar);

    return 0;
}
*/