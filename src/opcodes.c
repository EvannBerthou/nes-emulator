#include "opcodes.h"
#include "nes.h"
#include "utils.h"
#include <stdio.h>
#include <string.h>

#define SET_CARRY(v) set_carry_flag_value(nes->cpu, !!((v) & 0x80))

#define SET_ZERO(v) set_zero_flag_value(nes->cpu, (v) == 0)

// Verifie si les signes sont les mêmes
#define SET_OVERFLOW(v)                                                        \
    set_overflow_flag_value(                                                   \
        nes->cpu, ((operand ^ (v)) & ((v) ^ nes->cpu->register_a) & 0x80))

#define SET_NEGATIVE(v) set_negative_flag_value(nes->cpu, (v >> 7) & 1)

/*
 * OPCODES IMPLEMENTATION
 */

static void adc(NES *nes, addressing_mode mode) {
    uint16_t operand_addr = get_operand_addr(nes, mode);
    uint8_t operand = nes->cpu->memory[operand_addr];
    uint8_t carry_flag = nes->cpu->status & FLAG_CARRY;
    uint8_t result = (nes->cpu->register_a + operand + carry_flag) & 0xFF;

    SET_CARRY(result);
    SET_OVERFLOW(result);
    SET_NEGATIVE(result);

    nes->cpu->register_a = result;
}

static void and (NES * nes, addressing_mode mode) {
    uint16_t operand_addr = get_operand_addr(nes, mode);
    uint8_t operand = nes->cpu->memory[operand_addr];
    uint8_t result = nes->cpu->register_a & operand;

    SET_ZERO(result);
    SET_NEGATIVE(result);

    nes->cpu->register_a = result;
}

static void asl_acc(NES *nes, addressing_mode mode) {
    (void)mode;
    uint8_t result = nes->cpu->register_a << 1;

    SET_CARRY(nes->cpu->register_a);
    SET_ZERO(result);
    SET_NEGATIVE(result);

    nes->cpu->register_a = result;
}

static void asl(NES *nes, addressing_mode mode) {
    uint16_t operand_addr = get_operand_addr(nes, mode);
    uint8_t result = nes->cpu->memory[operand_addr] << 1;

    SET_CARRY(nes->cpu->register_a);
    SET_ZERO(result);
    SET_NEGATIVE(result);

    nes->cpu->memory[operand_addr] = result;
}

static void bcc(NES *nes, addressing_mode mode) {
    (void)mode;
    if (nes->cpu->carry == 0) {
        nes->cpu->pc = get_operand_addr(nes, mode);
    }
}

static void bcs(NES *nes, addressing_mode mode) {
    (void)mode;
    if (nes->cpu->carry == 1) {
        nes->cpu->pc = get_operand_addr(nes, mode);
    }
}

static void beq(NES *nes, addressing_mode mode) {
    (void)mode;
    if (nes->cpu->zero == 1) {
        nes->cpu->pc = get_operand_addr(nes, mode);
    }
}

static void bit(NES *nes, addressing_mode mode) {
    uint16_t operand_addr = get_operand_addr(nes, mode);
    uint8_t operand = nes->cpu->memory[operand_addr];

    SET_ZERO(nes->cpu->register_a & operand);
    nes->cpu->overflow = (operand >> 6) & 1;
    SET_NEGATIVE(operand);
}

static void bmi(NES *nes, addressing_mode mode) {
    (void)mode;
    if (nes->cpu->negative == 1) {
        nes->cpu->pc = get_operand_addr(nes, mode);
    }
}

static void bne(NES *nes, addressing_mode mode) {
    (void)mode;
    if (nes->cpu->zero == 0) {
        nes->cpu->pc = get_operand_addr(nes, mode);
    }
}

static void bpl(NES *nes, addressing_mode mode) {
    (void)mode;
    if (nes->cpu->negative == 0) {
        nes->cpu->pc = get_operand_addr(nes, mode);
    }
}

static void brk(NES *nes, addressing_mode mode) {
    // TODO: Implémenter
    (void)nes;
    (void)mode;
}

static void bvc(NES *nes, addressing_mode mode) {
    if (nes->cpu->overflow == 0) {
        nes->cpu->pc = get_operand_addr(nes, mode);
    }
}

static void bvs(NES *nes, addressing_mode mode) {
    if (nes->cpu->overflow == 1) {
        nes->cpu->pc = get_operand_addr(nes, mode);
    }
}

static void clc(NES *nes, addressing_mode mode) {
    (void)mode;
    nes->cpu->carry = 0;
}

static void cld(NES *nes, addressing_mode mode) {
    (void)mode;
    nes->cpu->decimal_mode = 0;
}

static void cli(NES *nes, addressing_mode mode) {
    (void)mode;
    nes->cpu->interupt_disable = 0;
}

static void clv(NES *nes, addressing_mode mode) {
    (void)mode;
    nes->cpu->overflow = 0;
}

static void cmp(NES *nes, addressing_mode mode) {
    uint16_t operand_addr = get_operand_addr(nes, mode);
    uint8_t operand = nes->cpu->memory[operand_addr];
    uint8_t result = nes->cpu->register_a - operand;

    nes->cpu->carry = nes->cpu->register_a >= operand;
    SET_ZERO(result);
    SET_NEGATIVE(result);
}

static void cpx(NES *nes, addressing_mode mode) {
    uint16_t operand_addr = get_operand_addr(nes, mode);
    uint8_t operand = nes->cpu->memory[operand_addr];
    uint8_t result = nes->cpu->register_x - operand;

    nes->cpu->carry = nes->cpu->register_x >= operand;
    SET_ZERO(result);
    SET_NEGATIVE(result);
}

static void cpy(NES *nes, addressing_mode mode) {
    uint16_t operand_addr = get_operand_addr(nes, mode);
    uint8_t operand = nes->cpu->memory[operand_addr];
    uint8_t result = nes->cpu->register_y - operand;

    nes->cpu->carry = nes->cpu->register_y >= operand;
    SET_ZERO(result);
    SET_NEGATIVE(result);
}

static void dec(NES *nes, addressing_mode mode) {
    uint16_t operand_addr = get_operand_addr(nes, mode);
    uint8_t result = nes->cpu->memory[operand_addr] - 1;

    SET_ZERO(result);
    SET_NEGATIVE(result);

    nes->cpu->memory[operand_addr] = result;
}

static void dex(NES *nes, addressing_mode mode) {
    (void)mode;
    nes->cpu->register_x--;
    SET_ZERO(nes->cpu->register_x);
    SET_NEGATIVE(nes->cpu->register_x);
}

static void dey(NES *nes, addressing_mode mode) {
    (void)mode;
    nes->cpu->register_y--;
    SET_ZERO(nes->cpu->register_y);
    SET_NEGATIVE(nes->cpu->register_y);
}

static void eor(NES *nes, addressing_mode mode) {
    uint16_t operand_addr = get_operand_addr(nes, mode);
    uint8_t operand = nes->cpu->memory[operand_addr];
    uint8_t result = nes->cpu->register_a ^ operand;

    SET_ZERO(result);
    SET_NEGATIVE(result);

    nes->cpu->register_a = result;
}

static void inc(NES *nes, addressing_mode mode) {
    uint16_t operand_addr = get_operand_addr(nes, mode);
    uint8_t result = nes->cpu->memory[operand_addr] + 1;

    SET_ZERO(result);
    SET_NEGATIVE(result);

    nes->cpu->memory[operand_addr] = result;
}

static void inx(NES *nes, addressing_mode mode) {
    (void)mode;
    nes->cpu->register_x++;
    SET_ZERO(nes->cpu->register_x);
    SET_NEGATIVE(nes->cpu->register_x);
}

static void iny(NES *nes, addressing_mode mode) {
    (void)mode;
    nes->cpu->register_y++;
    SET_ZERO(nes->cpu->register_y);
    SET_NEGATIVE(nes->cpu->register_y);
}

static void jmp(NES *nes, addressing_mode mode) {
    uint8_t low = nes->cpu->memory[nes->cpu->pc++];
    uint8_t high = nes->cpu->memory[nes->cpu->pc++];
    uint16_t addr = (high << 8) | low;
    if (mode == MODE_ABSOLUTE) {
        nes->cpu->pc = addr;
    } else if (mode == MODE_INDIRECT) {
        // TODO: Prendre en compte le bug
        nes->cpu->pc = nes->cpu->memory[addr];
    }
}

static void jsr(NES *nes, addressing_mode mode) {
    (void)mode;
    uint8_t low = nes->cpu->memory[nes->cpu->pc++];
    uint8_t high = nes->cpu->memory[nes->cpu->pc++];
    uint16_t addr = (high << 8) | low;
    stack_push(nes->cpu, nes->cpu->pc - 1);
    nes->cpu->pc = addr;
}

static void lda(NES *nes, addressing_mode mode) {
    uint16_t operand_addr = get_operand_addr(nes, mode);
    uint8_t operand = nes->cpu->memory[operand_addr];

    SET_ZERO(operand);
    SET_NEGATIVE(operand);

    nes->cpu->register_a = operand;
}

static void ldx(NES *nes, addressing_mode mode) {
    uint16_t operand_addr = get_operand_addr(nes, mode);
    uint8_t operand = nes->cpu->memory[operand_addr];

    SET_ZERO(operand);
    SET_NEGATIVE(operand);

    nes->cpu->register_x = operand;
}

static void ldy(NES *nes, addressing_mode mode) {
    uint16_t operand_addr = get_operand_addr(nes, mode);
    uint8_t operand = nes->cpu->memory[operand_addr];

    SET_ZERO(operand);
    SET_NEGATIVE(operand);

    nes->cpu->register_y = operand;
}

static void lsr_acc(NES *nes, addressing_mode mode) {
    (void)mode;
    uint8_t result = nes->cpu->register_a >> 1;

    SET_CARRY(nes->cpu->register_a & 1);
    SET_ZERO(result);
    SET_NEGATIVE(result);

    nes->cpu->register_a = result;
}

static void lsr(NES *nes, addressing_mode mode) {
    uint16_t operand_addr = get_operand_addr(nes, mode);
    uint8_t result = nes->cpu->memory[operand_addr] >> 1;

    SET_CARRY(nes->cpu->memory[operand_addr] & 1);
    SET_ZERO(result);
    SET_NEGATIVE(result);

    nes->cpu->memory[operand_addr] = result;
}

static void nop(NES *nes, addressing_mode mode) {
    (void)nes;
    (void)mode;
}

static void ora(NES *nes, addressing_mode mode) {
    uint16_t operand_addr = get_operand_addr(nes, mode);
    uint8_t operand = nes->cpu->memory[operand_addr];
    uint8_t result = nes->cpu->register_a | operand;

    SET_ZERO(result);
    SET_NEGATIVE(result);

    nes->cpu->register_a = result;
}

static void pha(NES *nes, addressing_mode mode) {
    (void)mode;
    stack_push(nes->cpu, nes->cpu->register_a);
}

static void php(NES *nes, addressing_mode mode) {
    (void)mode;
    stack_push(nes->cpu, nes->cpu->status);
}

static void pla(NES *nes, addressing_mode mode) {
    (void)mode;
    uint8_t stack_value = stack_pop(nes->cpu);
    SET_ZERO(stack_value);
    SET_NEGATIVE(stack_value);
    nes->cpu->register_a = stack_value;
}

static void plp(NES *nes, addressing_mode mode) {
    (void)mode;
    uint8_t stack_value = stack_pop(nes->cpu);
    SET_ZERO(stack_value);
    SET_NEGATIVE(stack_value);
    nes->cpu->status = stack_value;
}

static void rol_acc(NES *nes, addressing_mode mode) {
    (void)mode;
    uint8_t result = nes->cpu->register_a << 1;
    result |= nes->cpu->carry;

    SET_CARRY((nes->cpu->register_a >> 7) & 1);
    SET_ZERO(result);
    SET_NEGATIVE(result);

    nes->cpu->register_a = result;
}

static void rol(NES *nes, addressing_mode mode) {
    uint16_t operand_addr = get_operand_addr(nes, mode);
    uint8_t result = nes->cpu->memory[operand_addr] << 1;
    result |= nes->cpu->carry;

    SET_CARRY((nes->cpu->memory[operand_addr] >> 7) & 1);
    SET_ZERO(result);
    SET_NEGATIVE(result);

    nes->cpu->memory[operand_addr] = result;
}

static void ror_acc(NES *nes, addressing_mode mode) {
    (void)mode;
    uint8_t result = nes->cpu->register_a >> 1;
    result |= (nes->cpu->carry << 7);

    SET_CARRY(nes->cpu->register_a & 1);
    SET_ZERO(result);
    SET_NEGATIVE(result);

    nes->cpu->register_a = result;
}

static void ror(NES *nes, addressing_mode mode) {
    uint16_t operand_addr = get_operand_addr(nes, mode);
    uint8_t result = nes->cpu->memory[operand_addr] >> 1;
    result |= (nes->cpu->carry << 7);

    SET_CARRY(nes->cpu->memory[operand_addr] & 1);
    SET_ZERO(result);
    SET_NEGATIVE(result);

    nes->cpu->memory[operand_addr] = result;
}

static void rti(NES *nes, addressing_mode mode) {
    (void)mode;
    uint8_t status = stack_pop(nes->cpu);
    uint8_t pc = stack_pop(nes->cpu);
    nes->cpu->status = status;
    nes->cpu->pc = pc;
}

static void rts(NES *nes, addressing_mode mode) {
    (void)mode;
    uint8_t pc = stack_pop(nes->cpu);
    nes->cpu->pc = pc;
}

static void sbc(NES *nes, addressing_mode mode) {
    uint16_t operand_addr = get_operand_addr(nes, mode);
    uint8_t operand = nes->cpu->memory[operand_addr];
    uint8_t carry_flag = 1 - nes->cpu->carry;
    uint8_t result = (nes->cpu->register_a - operand + carry_flag) & 0xFF;

    SET_CARRY(result);
    SET_OVERFLOW(result);
    SET_NEGATIVE(result);

    nes->cpu->register_a = result;
}

static void sec(NES *nes, addressing_mode mode) {
    (void)mode;
    nes->cpu->carry = 1;
}

static void sed(NES *nes, addressing_mode mode) {
    (void)mode;
    nes->cpu->decimal_mode = 1;
}

static void sei(NES *nes, addressing_mode mode) {
    (void)mode;
    nes->cpu->interupt_disable = 1;
}

static void sta(NES *nes, addressing_mode mode) {
    uint16_t operand_addr = get_operand_addr(nes, mode);
    nes->cpu->memory[operand_addr] = nes->cpu->register_a;
}

static void stx(NES *nes, addressing_mode mode) {
    uint16_t operand_addr = get_operand_addr(nes, mode);
    nes->cpu->memory[operand_addr] = nes->cpu->register_x;
}

static void sty(NES *nes, addressing_mode mode) {
    uint16_t operand_addr = get_operand_addr(nes, mode);
    nes->cpu->memory[operand_addr] = nes->cpu->register_y;
}

static void tax(NES *nes, addressing_mode mode) {
    (void)mode;
    nes->cpu->register_x = nes->cpu->register_a;
    SET_ZERO(nes->cpu->register_x);
    SET_NEGATIVE(nes->cpu->register_x);
}

static void tay(NES *nes, addressing_mode mode) {
    (void)mode;
    nes->cpu->register_y = nes->cpu->register_a;
    SET_ZERO(nes->cpu->register_y);
    SET_NEGATIVE(nes->cpu->register_y);
}

static void tsx(NES *nes, addressing_mode mode) {
    (void)mode;
    uint8_t stack_value = stack_pop(nes->cpu);

    SET_ZERO(stack_value);
    SET_NEGATIVE(stack_value);

    nes->cpu->register_x = stack_value;
}

static void txa(NES *nes, addressing_mode mode) {
    (void)mode;
    nes->cpu->register_a = nes->cpu->register_x;
    SET_ZERO(nes->cpu->register_a);
    SET_NEGATIVE(nes->cpu->register_a);
}

static void txs(NES *nes, addressing_mode mode) {
    (void)mode;
    stack_push(nes->cpu, nes->cpu->register_x);
}

static void tya(NES *nes, addressing_mode mode) {
    (void)mode;
    nes->cpu->register_a = nes->cpu->register_y;
    SET_ZERO(nes->cpu->register_a);
    SET_NEGATIVE(nes->cpu->register_a);
}

/*
 * OPCODE DEFINITIONS
 */

opcode opcodes[256] = {
    {"NULL", MODE_IMPLIED, NULL},
    [0x69] = {"ADC", MODE_IMMEDIATE, &adc},
    [0x65] = {"ADC", MODE_ZERO_PAGE, &adc},
    [0x75] = {"ADC", MODE_ZERO_PAGE_X, &adc},
    [0x6D] = {"ADC", MODE_ABSOLUTE, &adc},
    [0x7D] = {"ADC", MODE_ABSOLUTE_X, &adc},
    [0x79] = {"ADC", MODE_ABSOLUTE_Y, &adc},
    [0x61] = {"ADC", MODE_INDIRECT_X, &adc},
    [0x71] = {"ADC", MODE_INDIRECT_Y, &adc},

    [0x29] = {"AND", MODE_IMMEDIATE, &and},
    [0x25] = {"AND", MODE_ZERO_PAGE, &and},
    [0x35] = {"AND", MODE_ZERO_PAGE_X, &and},
    [0x2D] = {"AND", MODE_ABSOLUTE, &and},
    [0x3D] = {"AND", MODE_ABSOLUTE_X, &and},
    [0x39] = {"AND", MODE_ABSOLUTE_Y, &and},
    [0x21] = {"AND", MODE_INDIRECT_X, &and},
    [0x31] = {"AND", MODE_INDIRECT_Y, &and},

    [0x0A] = {"ASL", MODE_ACCUMULATOR, &asl_acc},
    [0x06] = {"ASL", MODE_ZERO_PAGE, &asl},
    [0x16] = {"ASL", MODE_ZERO_PAGE_X, &asl},
    [0x0E] = {"ASL", MODE_ABSOLUTE, &asl},
    [0x1E] = {"ASL", MODE_ABSOLUTE_X, &asl},

    [0x90] = {"BCC", MODE_RELATIVE, &bcc},
    [0xB0] = {"BCS", MODE_RELATIVE, &bcs},
    [0xF0] = {"BEQ", MODE_RELATIVE, &beq},

    [0x24] = {"BIT", MODE_ZERO_PAGE, &bit},
    [0x2C] = {"BIT", MODE_ABSOLUTE, &bit},

    [0x30] = {"BMI", MODE_RELATIVE, &bmi},
    [0xD0] = {"BNE", MODE_RELATIVE, &bne},
    [0x10] = {"BPL", MODE_RELATIVE, &bpl},

    [0x00] = {"BRK", MODE_IMPLIED, &brk},

    [0x50] = {"BVC", MODE_RELATIVE, &bvc},
    [0x70] = {"BVS", MODE_RELATIVE, &bvs},

    [0x18] = {"CLC", MODE_IMPLIED, &clc},
    [0xD8] = {"CLD", MODE_IMPLIED, &cld},
    [0x58] = {"CLI", MODE_IMPLIED, &cli},
    [0xB8] = {"CLV", MODE_IMPLIED, &clv},

    [0xC9] = {"CMP", MODE_IMMEDIATE, &cmp},
    [0xC5] = {"CMP", MODE_ZERO_PAGE, &cmp},
    [0xD5] = {"CMP", MODE_ZERO_PAGE_X, &cmp},
    [0xCD] = {"CMP", MODE_ABSOLUTE, &cmp},
    [0xDD] = {"CMP", MODE_ABSOLUTE_X, &cmp},
    [0xD9] = {"CMP", MODE_ABSOLUTE_Y, &cmp},
    [0xC1] = {"CMP", MODE_INDIRECT_X, &cmp},
    [0xD1] = {"CMP", MODE_INDIRECT_Y, &cmp},

    [0xE0] = {"CPX", MODE_IMMEDIATE, &cpx},
    [0xE4] = {"CPX", MODE_ZERO_PAGE, &cpx},
    [0xEC] = {"CPX", MODE_ABSOLUTE, &cpx},

    [0xC0] = {"CPY", MODE_IMMEDIATE, &cpy},
    [0xC4] = {"CPY", MODE_ZERO_PAGE, &cpy},
    [0xCC] = {"CPY", MODE_ABSOLUTE, &cpy},

    [0xC6] = {"DEC", MODE_ZERO_PAGE, &dec},
    [0xD6] = {"DEC", MODE_ZERO_PAGE_X, &dec},
    [0xCE] = {"DEC", MODE_ABSOLUTE, &dec},
    [0xDE] = {"DEC", MODE_ABSOLUTE_X, &dec},
    [0xCA] = {"DEX", MODE_IMPLIED, &dex},
    [0x88] = {"DEY", MODE_IMPLIED, &dey},

    [0x49] = {"EOR", MODE_IMMEDIATE, &eor},
    [0x45] = {"EOR", MODE_ZERO_PAGE, &eor},
    [0x55] = {"EOR", MODE_ZERO_PAGE_X, &eor},
    [0x4D] = {"EOR", MODE_ABSOLUTE, &eor},
    [0x5D] = {"EOR", MODE_ABSOLUTE_X, &eor},
    [0x59] = {"EOR", MODE_ABSOLUTE_Y, &eor},
    [0x41] = {"EOR", MODE_INDIRECT_X, &eor},
    [0x51] = {"EOR", MODE_INDIRECT_Y, &eor},

    [0xE6] = {"INC", MODE_ZERO_PAGE, &inc},
    [0xF6] = {"INC", MODE_ZERO_PAGE_X, &inc},
    [0xEE] = {"INC", MODE_ABSOLUTE, &inc},
    [0xFE] = {"INC", MODE_ABSOLUTE_X, &inc},
    [0xE8] = {"INX", MODE_IMPLIED, &inx},
    [0xC8] = {"INY", MODE_IMPLIED, &iny},

    [0x4C] = {"JMP", MODE_ABSOLUTE, &jmp},
    [0x6C] = {"JMP", MODE_INDIRECT, &jmp},

    [0x20] = {"JSR", MODE_ABSOLUTE, &jsr},

    [0xA9] = {"LDA", MODE_IMMEDIATE, &lda},
    [0xA5] = {"LDA", MODE_ZERO_PAGE, &lda},
    [0xB5] = {"LDA", MODE_ZERO_PAGE_X, &lda},
    [0xAD] = {"LDA", MODE_ABSOLUTE, &lda},
    [0xBD] = {"LDA", MODE_ABSOLUTE_X, &lda},
    [0xB9] = {"LDA", MODE_ABSOLUTE_Y, &lda},
    [0xA1] = {"LDA", MODE_INDIRECT_X, &lda},
    [0xB1] = {"LDA", MODE_INDIRECT_Y, &lda},

    [0xA2] = {"LDX", MODE_IMMEDIATE, &ldx},
    [0xA6] = {"LDX", MODE_ZERO_PAGE, &ldx},
    [0xB6] = {"LDX", MODE_ZERO_PAGE_Y, &ldx},
    [0xAE] = {"LDX", MODE_ABSOLUTE, &ldx},
    [0xBE] = {"LDX", MODE_ABSOLUTE_Y, &ldx},

    [0xA0] = {"LDY", MODE_IMMEDIATE, &ldy},
    [0xA4] = {"LDY", MODE_ZERO_PAGE, &ldy},
    [0xB4] = {"LDY", MODE_ZERO_PAGE_X, &ldy},
    [0xAC] = {"LDY", MODE_ABSOLUTE, &ldy},
    [0xBC] = {"LDY", MODE_ABSOLUTE_X, &ldy},

    [0x4A] = {"LSR", MODE_ACCUMULATOR, &lsr_acc},
    [0x46] = {"LSR", MODE_ZERO_PAGE, &lsr},
    [0x56] = {"LSR", MODE_ZERO_PAGE_X, &lsr},
    [0x4E] = {"LSR", MODE_ABSOLUTE, &lsr},
    [0x5E] = {"LSR", MODE_ABSOLUTE_X, &lsr},

    [0xEA] = {"NOP", MODE_IMPLIED, &nop},

    [0x09] = {"ORA", MODE_IMMEDIATE, &ora},
    [0x05] = {"ORA", MODE_ZERO_PAGE, &ora},
    [0x15] = {"ORA", MODE_ZERO_PAGE_X, &ora},
    [0x0D] = {"ORA", MODE_ABSOLUTE, &ora},
    [0x1D] = {"ORA", MODE_ABSOLUTE_X, &ora},
    [0x19] = {"ORA", MODE_ABSOLUTE_Y, &ora},
    [0x01] = {"ORA", MODE_INDIRECT_X, &ora},
    [0x11] = {"ORA", MODE_INDIRECT_Y, &ora},

    [0x48] = {"PHA", MODE_IMPLIED, &pha},
    [0x08] = {"PHP", MODE_IMPLIED, &php},
    [0x68] = {"PLA", MODE_IMPLIED, &pla},
    [0x28] = {"PLP", MODE_IMPLIED, &plp},

    [0x2A] = {"ROL", MODE_ACCUMULATOR, &rol_acc},
    [0x26] = {"ROL", MODE_ZERO_PAGE, &rol},
    [0x36] = {"ROL", MODE_ZERO_PAGE_X, &rol},
    [0x2E] = {"ROL", MODE_ABSOLUTE, &rol},
    [0x3E] = {"ROL", MODE_ABSOLUTE_X, &rol},

    [0x6A] = {"ROR", MODE_ACCUMULATOR, &ror_acc},
    [0x66] = {"ROR", MODE_ZERO_PAGE, &ror},
    [0x76] = {"ROR", MODE_ZERO_PAGE_X, &ror},
    [0x6E] = {"ROR", MODE_ABSOLUTE, &ror},
    [0x7E] = {"ROR", MODE_ABSOLUTE_X, &ror},

    [0x40] = {"RTI", MODE_IMPLIED, &rti},
    [0x60] = {"RTS", MODE_IMPLIED, &rts},

    [0xE9] = {"SBC", MODE_IMMEDIATE, &sbc},
    [0xE5] = {"SBC", MODE_ZERO_PAGE, &sbc},
    [0xF5] = {"SBC", MODE_ZERO_PAGE_X, &sbc},
    [0xED] = {"SBC", MODE_ABSOLUTE, &sbc},
    [0xFD] = {"SBC", MODE_ABSOLUTE_X, &sbc},
    [0xF9] = {"SBC", MODE_ABSOLUTE_Y, &sbc},
    [0xE1] = {"SBC", MODE_INDIRECT_X, &sbc},
    [0xF1] = {"SBC", MODE_INDIRECT_Y, &sbc},

    [0x38] = {"SEC", MODE_IMPLIED, &sec},
    [0xF8] = {"SED", MODE_IMPLIED, &sed},
    [0x78] = {"SEI", MODE_IMPLIED, &sei},

    [0x85] = {"STA", MODE_ZERO_PAGE, &sta},
    [0x95] = {"STA", MODE_ZERO_PAGE_X, &sta},
    [0x8D] = {"STA", MODE_ABSOLUTE, &sta},
    [0x9D] = {"STA", MODE_ABSOLUTE_X, &sta},
    [0x99] = {"STA", MODE_ABSOLUTE_Y, &sta},
    [0x81] = {"STA", MODE_INDIRECT_X, &sta},
    [0x91] = {"STA", MODE_INDIRECT_Y, &sta},

    [0x86] = {"STX", MODE_ZERO_PAGE, &stx},
    [0x96] = {"STX", MODE_ZERO_PAGE_Y, &stx},
    [0x8E] = {"STX", MODE_ABSOLUTE, &stx},

    [0x84] = {"STY", MODE_ZERO_PAGE, &sty},
    [0x94] = {"STY", MODE_ZERO_PAGE_X, &sty},
    [0x8C] = {"STY", MODE_ABSOLUTE, &sty},

    [0xAA] = {"TAX", MODE_IMPLIED, &tax},
    [0xA8] = {"TAY", MODE_IMPLIED, &tay},
    [0xBA] = {"TSX", MODE_IMPLIED, &tsx},
    [0x8A] = {"TXA", MODE_IMPLIED, &txa},
    [0x9A] = {"TXS", MODE_IMPLIED, &txs},
    [0x98] = {"TYA", MODE_IMPLIED, &tya},
};

void exec_opcode(NES *nes, uint8_t code) {
    opcode *op = &opcodes[code];
    if (op->mnemonic == NULL) {
        PANIC("Invalid opcode %x\n", code);
    }
    //TODO: ??
    if (strcmp(op->mnemonic, "JSR") != 0 && strcmp(op->mnemonic, "JMP") != 0) {
        nes->cpu->pc++;
    } else {
        printf("JUMP\n");
    }
    op->fct(nes, op->mode);
}
