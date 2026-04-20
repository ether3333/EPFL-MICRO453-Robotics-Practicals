MAIN = ex3

# What program(s) have to be built
PROGRAMS = $(MAIN)

# Dependencies for the program(s) to build
$(MAIN): ../common/remregs.o ../common/wperror.o ../common/robot.o ../common/utils.o $(MAIN).o

# Includes the common Makefile with the various rules
include ../common/Makefile.inc