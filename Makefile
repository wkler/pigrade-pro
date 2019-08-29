DEPS = src
PHONY = $(DEPS) clean mrproper

all: $(DEPS)
	mv src/pigrade-server .

$(DEPS):
	$(MAKE) -C $@ $(MAKECMDGOALS)

clean: $(DEPS)

mrproper: $(DEPS)
	rm -rf pigrade-server *~

.PHONY: $(PHONY)
