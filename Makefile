.PHONY: all
all: docs

.PHONY: docs
docs:
	doxygen Doxyfile
