CFLAGS = -Wall -g -Wextra -Wdouble-promotion -Wmissing-include-dirs -Wswitch-default -Wswitch-enum -c -fPIC -I.. -I.
LDFLAGS = -lpthread -lncurses
OBJECTS = main.o array.o kbhit.o progressbar.o statusbar.o
OUTNAME = pigrade

all: $(OUTNAME)

$(OUTNAME): $(OBJECTS)
	gcc -g -Wall -o $@ $(OBJECTS) $(LDFLAGS)

clean:
	rm -rf $(OBJECTS) $(OUTNAME)

mrproper:
	rm -rf $(OBJECTS) $(OUTNAME) *~

.PHONY: clean mrproper
