#!/bin/bash

fpc src/ufr.pp

sudo mkdir -p /usr/lib/x86_64-linux-gnu/fpc/3.2.2/units/x86_64-linux/ufr
sudo cp src/ufr.o src/ufr.ppu /usr/lib/x86_64-linux-gnu/fpc/3.2.2/units/x86_64-linux/ufr/

# sudo cp src/ufr.o src/ufr.ppu   /usr/lib/x86_64-linux-gnu/fpc/3.2.2/units/x86_64-linux/rtl-extra/

# fpc examples/pub_ii.pas

# Erro -> /usr/bin/ld.bfd: cannot find -lufr: No such file or directory
# falta o ufr.so