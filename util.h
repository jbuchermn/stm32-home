#ifndef UTIL_H
#define UTIL_H

#include <stdbool.h>

void halt(const char *msg);

void assertm(bool condition, const char *msg);

#define assert(c) assertm(c, #c)

#endif
