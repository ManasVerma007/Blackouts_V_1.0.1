#include <stdio.h>
#include <string.h>
#include <sys/unistd.h>
#include <sys/stat.h>

#define MAX_PORTS 8

typedef struct
{
    int port;
    int pixelCount;
    int startChannel;
    char colorOrder[4];
    int nulls;
    int brightness;
    float gamma;
} port_data;

extern port_data *ports[MAX_PORTS];