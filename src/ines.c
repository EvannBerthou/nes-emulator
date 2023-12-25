#include "ines.h"
#include "utils.h"
#include <stdio.h>
#include <string.h>

void load_rom(cartridge *cart, const char *rom_path) {
    FILE *file = fopen(rom_path, "r");
    ASSERT(file);

    char nes_header[4];
    fread(nes_header, sizeof(char), 4, file);
    const char expected_header[4] = {0x4E, 0x45, 0x53, 0x1A};
    if (strncmp(nes_header, expected_header, 4)) {
        PANIC("Header non trouvable pour la rom %s, %3s", rom_path, nes_header);
    }
    memcpy(cart->header.NES, nes_header, sizeof(char) * 3);
    ASSERT(fread(&cart->header.prg_size, sizeof(uint8_t), 1, file));
    ASSERT(fread(&cart->header.chr_size, sizeof(uint8_t), 1, file));
    ASSERT(fread(&cart->header.flag6, sizeof(uint8_t), 1, file));
    ASSERT(fread(&cart->header.flag7, sizeof(uint8_t), 1, file));
    ASSERT(fread(&cart->header.flag8, sizeof(uint8_t), 1, file));
    ASSERT(fread(&cart->header.flag9, sizeof(uint8_t), 1, file));
    ASSERT(fread(&cart->header.flag10, sizeof(uint8_t), 1, file));
    ASSERT(fread(&cart->header.flag10, sizeof(uint8_t), 1, file));

    ASSERT(!fseek(file, 4, SEEK_CUR));

    // TODO: Calculer le mapper

    // TODO: Prendre en compte trainer
    //uint8_t trainer;
    //ASSERT(fread(&trainer, sizeof(uint8_t), 1, file));

    cart->prg_data = malloc(0x4000 * cart->header.prg_size);
    ASSERT(cart->prg_data);
    ASSERT(fread(cart->prg_data, 1, 0x4000 * cart->header.prg_size, file));

    if (cart->header.chr_size) {
        cart->chr_data = malloc(0x2000 * cart->header.chr_size);
        ASSERT(cart->chr_data);
        ASSERT(fread(cart->chr_data, 1, 0x2000 * cart->header.chr_size, file));
    } else {
        cart->chr_data = NULL;
    }

    // TODO: Lire inst_rom et prom

    printf("ROM chargée !\n");

    fclose(file);
}
