#ifndef OPCODES_H
#define OPCODES_H

#include "cpu.h"

typedef struct {
    const char *mnemonic;
    addressing_mode mode;
    void (*fct)(cpu*, addressing_mode);
} opcode;

void exec_opcode(cpu *cpu, uint8_t code);

#endif
