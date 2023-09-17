#include "cpu.h"
#include <assert.h>
#include <stdio.h>

void init_cpu(cpu *cpu) {
    cpu->register_a = 0;
    cpu->register_x = 0;
    cpu->status = 0;
    cpu->pc = 0;
}

uint8_t read_instruction(cpu *cpu) { return cpu->memory[cpu->pc++]; }

static uint8_t mem_read(cpu *cpu) { return cpu->memory[cpu->pc++]; }

static uint16_t mem_read_u16(cpu *cpu) {
    const uint16_t high = (cpu->memory[cpu->pc++] << 8);
    const uint8_t low = (cpu->memory[cpu->pc++]);
    return high | low;
}

uint16_t get_operand_addr(cpu *cpu, addressing_mode mode) {
    switch (mode) {
    case MODE_IMMEDIATE:
        return cpu->pc++;
    case MODE_ZERO_PAGE:
        return mem_read(cpu);
    case MODE_ABSOLUTE:
        return mem_read_u16(cpu);
    default:
        return 0;
    }
}

void set_carry_flag_value(cpu *cpu, uint8_t value) { cpu->carry = value != 0; }

void set_overflow_flag_value(cpu *cpu, uint8_t value) {
    cpu->overflow = value != 0;
}

void set_negative_flag_value(cpu *cpu, uint8_t value) {
    cpu->negative = value != 0;
}
