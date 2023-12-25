#!/bin/sh

gcc -Wall -Werror $(find src -name *.c) -o build -g
