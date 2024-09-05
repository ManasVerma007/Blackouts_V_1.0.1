#ifndef COMMON_H
#define COMMON_H

#include <stdbool.h> // For bool type

// Define an enum for show states
typedef enum {
    SHOW_NOT_COMPLETE,
    SHOW_COMPLETE
} ShowState;

typedef struct
{
    int showNumber;    
    char fseqFileName[50];
    bool repeat; 
    int nextShowNumber;
    ShowState state; 
} ShowData;

extern ShowData *shows;  
extern int globalShowCount; 
extern int buttonNumber; 

#endif