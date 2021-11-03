Import("env")

# original Makefile builds into dapboot.bin/elf, let's do the same
env.Replace(PROGNAME="dapboot")
