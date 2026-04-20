CPP = g++
CPPFLAGS = -Wall -O3 -I ../common -I ../../common

MSG_LINKING = " [link]    "
MSG_COMPILINGCPP = " [cpp]     "
MSG_CLEANING = " [clean]   "

# Comment compiler un .cc ou .cpp vers un .o
%.o: %.cc
	@echo $(MSG_COMPILINGCPP) $<
	@$(CPP) $(CPPFLAGS) -o $@ -c $<

%.o: %.cpp
	@echo $(MSG_COMPILINGCPP) $<
	@$(CPP) $(CPPFLAGS) -o $@ -c $<

# Comment faire un exécutable
%:
	@echo $(MSG_LINKING) $@
	@$(CPP) $(CPPFLAGS) -o $@ $^ $(LIBS)
	@strip $(addsuffix .exe,$@)

clean:
	@echo -n $(MSG_CLEANING)
	@-rm -f *.o
	@-rm -f ${PROGRAMS}
	@-rm -f ../common/*.o
	@-rm -f $(addsuffix .exe,${PROGRAMS})
	@echo done.

rebuild: clean all

all: ${PROGRAMS}
  
# Liste des "faux" targets (i.e. qui ne sont pas des fichiers)
.PHONY:		clean rebuild all
