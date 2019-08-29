DEPS = src
PHONY = $(DEPS) clean mrproper

all: $(DEPS)
	mv src/pigrade .

$(DEPS):
	$(MAKE) -C $@ $(MAKECMDGOALS)

clean: $(DEPS)

mrproper: $(DEPS)
	rm -rf pigrade *~

.PHONY: $(PHONY)
