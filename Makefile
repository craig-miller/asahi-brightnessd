PROG    = asahi-brightnessd
PREFIX ?= /usr
SBIN   ?= $(PREFIX)/sbin

CFLAGS  ?= -O2 -Wall -Wextra -Wpedantic
LDFLAGS ?=

$(PROG): $(PROG).c
	$(CC) $(CFLAGS) -o $@ $< $(LDFLAGS)

clean:
	rm -f $(PROG)

install: $(PROG)
	install -Dm755 $(PROG) $(DESTDIR)$(SBIN)/$(PROG)

.PHONY: clean install
