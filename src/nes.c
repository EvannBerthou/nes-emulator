#include "nes.h"
#include "utils.h"

uint8_t read_instruction(NES *nes) { return mem_read(nes, nes->cpu->pc); }

uint8_t mem_read(NES *nes, uint16_t addr) {
    if (addr < 0x4000) {
        return nes->cpu->memory[addr];
    }

    // TODO: Mapper
    if (addr < 0xC000)
        return nes->cart->prg_data[addr & 0x3fff];
    else {
        uint8_t *bank2 =
            nes->cart->prg_data + (nes->cart->header.prg_size - 1) * 0x4000;
        return bank2[addr & 0x3fff];
    }

    PANIC("Addresse invalide %x\n", addr);
    return 0;
}

// TODO: Bug ici
uint16_t mem_read_u16(NES *nes, uint16_t addr) {
    return mem_read(nes, addr) | mem_read(nes, addr + 1) << 8;
}

uint16_t get_operand_addr(NES *nes, addressing_mode mode) {
    cpu *cpu = nes->cpu;
    switch (mode) {
    case MODE_IMMEDIATE:
        return mem_read(nes, cpu->pc++);
    case MODE_RELATIVE: {
        int8_t offset = mem_read(nes, cpu->pc++);
        return cpu->pc + offset;
    }
    case MODE_ZERO_PAGE:
        return mem_read(nes, (cpu->pc++) & 0xFF);
    case MODE_ZERO_PAGE_X:
        return mem_read(nes, (++cpu->pc + cpu->register_x) & 0xFF);
    case MODE_ZERO_PAGE_Y:
        return mem_read(nes, (++cpu->pc + cpu->register_y) & 0xFF);
    case MODE_ABSOLUTE: {
        uint16_t ret = mem_read_u16(nes, cpu->pc++);
        cpu->pc++;
        return ret;
    }
    case MODE_ABSOLUTE_X:
        return mem_read_u16(nes, (cpu->pc++) + cpu->register_x);
    case MODE_ABSOLUTE_Y:
        return mem_read_u16(nes, (cpu->pc++) + cpu->register_y);
    case MODE_INDIRECT_X: {
        // val = PEEK(PEEK((arg + X) % 256) + PEEK((arg + X + 1) % 256) * 256)
        uint8_t x1 = mem_read(nes, (cpu->pc++) + cpu->register_x) & 0xFF;
        uint8_t x2 = (mem_read(nes, (cpu->pc++) + cpu->register_x) & 0xFF) << 8;
        return mem_read_u16(nes, x1 + x2);
    }
    case MODE_INDIRECT_Y: {
        // val = PEEK(PEEK(arg) + PEEK((arg + 1) % 256) * 256 + Y)
        uint8_t y1 = mem_read(nes, cpu->pc++);
        uint8_t y2 = ((mem_read(nes, cpu->pc++) & 0xFF) << 8) + cpu->register_y;
        return mem_read_u16(nes, y1 + y2);
    }
    default:
        fprintf(stderr, "[ERROR] Mode invalide: %d", mode);
        return 0;
    }
}

void reset_nes(NES *nes) {
    nes->cpu->pc = mem_read_u16(nes, RESET_VECTOR);
}
