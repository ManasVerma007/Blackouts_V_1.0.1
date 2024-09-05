// correction_arrays.h
#ifndef CORRECTION_ARRAYS_H
#define CORRECTION_ARRAYS_H

#define MAX_PORTS 8
#define MAX_VALUES 256

extern unsigned char gammaCorrection[MAX_PORTS][MAX_VALUES];
extern unsigned char brightnessAdjustment[MAX_PORTS][MAX_VALUES];

void populateCorrectionArrays(void);

#endif // CORRECTION_ARRAYS_H