MAIN = experiment

PROGRAMS = $(MAIN)

LIBS = -lwsock32

$(MAIN): ../common/remregs.o ../common/wperror.o ../common/robot.o \
	../common/utils.o ../common/netutil.o ../common/trkcli.o $(MAIN).o

include ../common/Makefile.inc
