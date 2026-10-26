# Schémas de brochage des cartes Nucleo, générés avec pinout
# (https://pinout.readthedocs.io/). pinout et cairosvg sont fournis par uv,
# sans installation globale. Sinon : make PINOUT="python -m pinout.manager"

PINOUT ?= uv run -q --no-project --with pinout --with cairosvg python -m pinout.manager
BOARDS := f446 f411 l476
DEPS := nucleo64.py styles.css

all: $(BOARDS:=.svg) $(BOARDS:=.png)

%.svg: %.py $(DEPS)
	@echo "Generating $@ from $<"
	$(PINOUT) --export $< $@ -o > /dev/null

%.png: %.py $(DEPS)
	@echo "Generating $@ from $<"
	$(PINOUT) --export $< $@ -o > /dev/null

clean:
	rm -f $(BOARDS:=.svg) $(BOARDS:=.png)
	rm -rf __pycache__

.PHONY: all clean
