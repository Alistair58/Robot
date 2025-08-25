#ifndef GLOBALS_H
#define GLOBALS_H

#include <errno.h>
#include <stdlib.h>
#include <stdio.h>

#define min(a,b) (((a)>(b))?(b):(a))
#define max(a,b) (((a)>(b))?(a):(b))
#define fcmp(a,b) ((a)+0.000001f>(b) && (a)-0.000001f<(b))
#define calloc_error() char error_msg[20+sizeof(__func__)]; \
    sprintf(error_msg,"\nCalloc error in \"%s\"",__func__); \
    errno = ENOMEM; \
    perror(error_msg); \
    exit(PICO_ERROR_INSUFFICIENT_RESOURCES);
extern int connected;
extern bool auto_mode;
extern bool core1_running;

#endif