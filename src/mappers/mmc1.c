#include "../ines.h"
#include "mapper.h"
#include <stddef.h>

static uint8_t bank = 0;

// TODO: Ne pas utiliser cartdrige
static void mmc1_write_prg(cartridge *cart, uint16_t addr, uint8_t val) {
    (void)cart;
    if (addr >= 0x8000) {
        bank = val & 0x7;
    }
}

static uint8_t mmc1_read_prg(cartridge *cart, uint16_t addr) {
    if (addr < 0xC000)
        return cart->prg_data[addr & 0x3fff];
    else {
        uint8_t *bank2 = cart->prg_data + (cart->header.prg_size - 1) * 0x4000;
        return bank2[addr & 0x3fff];
    }
}

//TODO: CHR pour le picturebus
mapper get_mmc1_mapper() {
    return (mapper){.write_prg = mmc1_write_prg,
                    .read_prg = mmc1_read_prg,
                    .write_chr = NULL,
                    .read_chr = NULL};
}
