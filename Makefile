pcf_file = res/io.pcf
ICELINK_DIR = E:\
# ICELINK_DIR=$(shell df | grep iCELink | awk '{print $$6}')
# ICELINK_DIR = /Volumes/iCELink
${warning iCELink path: $(ICELINK_DIR)}

filename = ./src/datapath.v
bench_target = ./test/soc_tb.v
bench_out = ./build/bench.vvp

all: bench wave

data: bin/instr.S
	clang -target riscv32 -march=rv32i -c bin/instr.S -o build/prog.o
	ld.lld -Tbin/ram.ld build/prog.o -o build/prog.elf
	llvm-readobj -a build/prog.elf > build/prog.header
	llvm-objdump -d build/prog.elf > build/prog.asm
	llvm-objcopy -O binary build/prog.elf build/prog.bin
	xxd -p -c 4 build/prog.bin > build/prog.hex

cdata: bin/prog.c
	clang -target riscv32 -march=rv32i -o build/entry.o -c bin/prog.S
	clang -target riscv32 -march=rv32i -o build/prog.o -c bin/prog.c
	ld.lld -Tbin/ram.ld build/entry.o build/prog.o -o build/prog.elf
	llvm-readobj -a build/prog.elf > build/prog.header
	llvm-objdump -d build/prog.elf > build/prog.asm
	llvm-objcopy -O binary build/prog.elf build/prog.bin
	xxd -p -c 4 build/prog.bin > build/prog.hex


.PHONY: bench wave build env flash

env:
	D:/Tools/oss-cad-suite/environment.bat

bench: $(bench_target)
	@mkdir -p build
	iverilog -DBENCH -y ./src -y ./comp  -o $(bench_out) $(bench_target) 

wave: $(bench_out)
	vvp -n $(bench_out)

debug: build/bench.vvp
	gtkwave build/bench.vcd &

build:
	@mkdir -p build
	yosys -p "hierarchy -top soc; synth_ice40 -json build/sys.json" src/*.v
	nextpnr-ice40 \
		--up5k \
		--package sg48 \
		--json build/sys.json \
		--pcf $(pcf_file) \
		--asc build/pnr.asc
		# --force
	icepack build/pnr.asc build/prog.bin

flash:
	@if [ -d '$(ICELINK_DIR)' ]; \
     	then \
		cp build/prog.bin $(ICELINK_DIR); \
     	else \
		echo "iCELink not found"; \
		exit 1; \
     	fi

clean:
	rm -rf build