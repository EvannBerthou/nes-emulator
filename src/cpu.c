#include "cpu.h"

void init_cpu(cpu *cpu) {
    cpu->register_a = 0;
    cpu->register_x = 0;
    cpu->status = 0;
    cpu->pc = 0;
    cpu->sp = 0xFF;
}

void stack_push(cpu *cpu, uint8_t value) {
    cpu->memory[cpu->sp] = value;
    cpu->sp--;
}

uint8_t stack_pop(cpu *cpu) {
    cpu->sp++;
    return cpu->memory[cpu->sp];
}

void set_carry_flag_value(cpu *cpu, uint8_t value) { cpu->carry = value != 0; }

void set_zero_flag_value(cpu *cpu, uint8_t value) { cpu->zero = value; }

void set_overflow_flag_value(cpu *cpu, uint8_t value) {
    cpu->overflow = value != 0;
}

void set_negative_flag_value(cpu *cpu, uint8_t value) {
    cpu->negative = value != 0;
}
