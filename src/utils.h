#ifndef UTILS_H
#define UTILS_H

#include <stdio.h>
#include <stdlib.h>

#define PANIC(fmt, ...) do { fprintf(stderr, (fmt), __VA_ARGS__); exit(1); } while(0); 

#define ASSERT(x) if (!x) {PANIC("Erreur %s:%d", __FILE__, __LINE__);}

#endif
