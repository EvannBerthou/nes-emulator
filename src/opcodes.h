#ifndef OPCODES_H
#define OPCODES_H

#include "cpu.h"
#include "nes.h"

typedef struct {
    const char *mnemonic;
    addressing_mode mode;
    void (*fct)(NES*, addressing_mode);
} opcode;

void exec_opcode(NES *nes, uint8_t code);

#endif
