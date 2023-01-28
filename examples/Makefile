all: clean shtc1_linux

CFLAGS += -I..

shtc1_linux: linux_userspace.c ../shtc1.c
	$(CC) $(CFLAGS) -o $@ $^
	strip $@

clean:
	rm -f shtc1_linux
