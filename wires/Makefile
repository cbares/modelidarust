
SOURCES := $(wildcard *.yml)
TARGETS := $(SOURCES:.yml=.png)

all: $(TARGETS)

%.png: %.yml
	@echo "Generating $@ from $<"
	wireviz $< -f s