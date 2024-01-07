#ifndef MAPPER_H
#define MAPPER_H

#include <stdint.h>
#include "../ines.h"

typedef void (*write_fct)(cartridge*, uint16_t, uint8_t);
typedef uint8_t (*read_fct)(cartridge*, uint16_t);

typedef struct {
    write_fct write_prg;
    read_fct read_prg;
    write_fct write_chr;
    read_fct read_chr;
} mapper;

mapper get_mmc1_mapper();

#endif
