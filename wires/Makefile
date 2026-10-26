
SOURCES := $(wildcard *.yml)
TARGETS := $(SOURCES:.yml=.png) $(SOURCES:.yml=.svg)

all: $(TARGETS)

%.png: %.yml
	@echo "Generating $@ from $<"
	wireviz $< -f p

%.svg: %.yml
	@echo "Generating $@ from $<"
	wireviz $< -f s
