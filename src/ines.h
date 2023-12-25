#ifndef INES_H
#define INES_H

#include <stdint.h>

typedef struct {
    char NES[3]; // "NES" Header
    uint8_t prg_size; //16KB units
    uint8_t chr_size; //8KB units TODO: Peut être 0 => cas spécial
    uint8_t flag6; //TODO: Comprendre
    uint8_t flag7;
    uint8_t flag8;
    uint8_t flag9;
    uint8_t flag10;
    uint8_t unused[5];
} rom_header;

typedef struct {
    rom_header header;
    uint8_t *trainer;
    uint8_t *prg_data;
    uint8_t *chr_data;
    uint8_t *inst_rom;
    uint8_t *prom; //Often missing
} cartridge;

void load_rom(cartridge *cart, const char *rom_path);

#endif
