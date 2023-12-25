#ifndef NES_H
#define NES_H

#include "cpu.h"
#include "ines.h"

typedef struct {
    cpu *cpu;
    cartridge *cart;
} NES;

uint8_t read_instruction(NES *nes);
uint8_t mem_read(NES *nes, uint16_t addr);
uint16_t mem_read_u16(NES *nes, uint16_t addr);
uint16_t get_operand_addr(NES *nes, addressing_mode mode);
void reset_nes(NES *nes);

#endif
