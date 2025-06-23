#ifndef GLOBALS_H
#define GLOBALS_H

#define min(a,b) (((a)>(b))?(b):(a))
#define fcmp(a,b) ((a)+0.000001f>(b) && (a)-0.000001f<(b))
extern int connected;
extern bool auto_mode;

#endif