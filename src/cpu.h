#ifndef CPU_H
#define CPU_H

#include <stdint.h>

typedef struct {
    uint8_t register_a;
    uint8_t register_x;
    union {
        struct {
            uint8_t carry : 1;
            uint8_t zero : 1;
            uint8_t interupt_disable : 1;
            uint8_t decimal_mode : 1;
            uint8_t break1 : 1;
            uint8_t break2 : 1;
            uint8_t overflow : 1;
            uint8_t negative : 1;
        };
        uint8_t status;
    };
    uint16_t pc;
    uint8_t memory[1 << 16];
} cpu;

typedef enum {
    MODE_IMMEDIATE,
    MODE_ZERO_PAGE,
    MODE_ZERO_PAGE_X,
    MODE_ZERO_PAGE_Y,
    MODE_ABSOLUTE,
    MODE_ABSOLUTE_X,
    MODE_ABSOLUTE_Y,
    MODE_INDIRECT_X,
    MODE_INDIRECT_Y,
    MODE_NONE
} addressing_mode;

typedef enum {
    FLAG_CARRY = 0x1,
    FLAG_ZERO = 0x2,
    FLAG_INTERUPT_DISABLE = 0x4,
    FLAG_DECIMAL_MODE = 0x8,
    FLAG_BREAK = 0x10,
    FLAG_BREAK2 = 0x20,
    FLAG_OVERFLOW = 0x40,
    FLAG_NEGATIVE = 0x80,
} flags;

void init_cpu(cpu *cpu);
uint8_t read_instruction(cpu *cpu);
uint16_t get_operand_addr(cpu *cpu, addressing_mode mode);

void set_carry_flag_value(cpu *cpu, uint8_t value);
void set_overflow_flag_value(cpu *cpu, uint8_t value);
void set_negative_flag_value(cpu *cpu, uint8_t value);

#endif
