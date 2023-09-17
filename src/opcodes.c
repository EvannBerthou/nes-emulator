#include <stdio.h>
#include "opcodes.h"

/*
 * OPCODES IMPLEMENTATION
 */

void adc(cpu *cpu, addressing_mode mode) {
    uint8_t operand_addr = get_operand_addr(cpu, mode);
    uint8_t operand = cpu->memory[operand_addr];
    uint8_t carry_flag = cpu->status & FLAG_CARRY;
    uint16_t sum = cpu->register_a + operand + carry_flag;

    uint8_t result = sum & 0xFF;
    
    set_carry_flag_value(cpu, sum > 0xFF);
    set_overflow_flag_value(cpu, ((operand ^ result) & (result ^ cpu->register_a) & 0x80) != 0);
    set_negative_flag_value(cpu, result & (1 << 7));

    cpu->register_a = result;
}

/*
 * OPCODE DEFINITIONS
 */

opcode opcodes[256] = {
    [0x69] = {"ADC", MODE_IMMEDIATE, &adc},
    [0x65] = {"ADC", MODE_IMMEDIATE, &adc},
    [0x75] = {"ADC", MODE_IMMEDIATE, &adc},
    [0x6D] = {"ADC", MODE_IMMEDIATE, &adc},
    [0x7D] = {"ADC", MODE_IMMEDIATE, &adc},
    [0x61] = {"ADC", MODE_IMMEDIATE, &adc},
    [0x71] = {"ADC", MODE_IMMEDIATE, &adc},
};

void exec_opcode(cpu *cpu, uint8_t code) {
    opcode *op = &opcodes[code];
    op->fct(cpu, op->mode);
}
