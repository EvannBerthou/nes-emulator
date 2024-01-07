#!/bin/sh

gcc -Wall -Werror -Wunused-variable -Wunused-parameter $(find src -name *.c) -o build -g
