# What program(s) have to be built
PROGRAMS = ex6

# Libraries needed for the executable file
LIBS = -lwsock32

# Task 6.1: tracking client + remote registers (head LED) in one executable
ex6: ../common/netutil.o ../common/wperror.o ../common/trkcli.o ../common/utils.o \
	../common/remregs.o ../common/robot.o ex6.o

# Includes the common Makefile with the various rules
include ../common/Makefile.inc
