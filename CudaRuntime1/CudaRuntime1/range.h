#pragma once

#include <stdlib.h>

typedef struct
{
    float start, end, step;
} range;

range* newRange(float, float, float);

int sizeFromRange(range*);