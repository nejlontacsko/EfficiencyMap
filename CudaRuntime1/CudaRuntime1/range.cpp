#include "range.h"

range* newRange(float start, float end, float step)
{
    range* r = (range*)malloc(sizeof(range));

    r->start = start;
    r->end = end;
    r->step = step;

    return r;
}

int sizeFromRange(range* r)
{
    return r->end - r->start + 1;
}