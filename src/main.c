#include "cpu.h"
#include "ines.h"
#include "mappers/mapper.h"
#include "nes.h"
#include "opcodes.h"

int main() {
    cpu c;
    init_cpu(&c);

    cartridge cart;
    load_rom(&cart, "zelda.nes");

    mapper m = get_mmc1_mapper();
    NES nes = {&c, &cart, &m};
    reset_nes(&nes);

    while (1) {
        uint8_t code = read_instruction(&nes);
        exec_opcode(&nes, code);
    }

    return 0;
}
