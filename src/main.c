#include "cpu.h"
#include "opcodes.h"
#include <stdio.h>
#include <stdlib.h>

int main() {
    cpu c;
    init_cpu(&c);
    c.memory[0] = 0x69;
    c.memory[1] = 0xFF;
    c.memory[2] = 0x69;
    c.memory[3] = 0x0F;

    while (1) {
        uint8_t code = read_instruction(&c);
        if (code == 0) exit(1);

        exec_opcode(&c, code);
        printf("%d %d\n", c.negative, c.register_a);
    }

    return 0;
}
