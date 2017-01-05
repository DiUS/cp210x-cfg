vpath %.c src

OBJS=\
  build/main.o

CFLAGS=-std=c11 -O2 -pipe -Wall -Wextra -D_POSIX_C_SOURCE=2 -c
LDFLAGS=-lusb-1.0

build/%.o: %.c
	$(CC) $(CFLAGS) $< -o $@

cp210x-cfg: $(OBJS)
	$(CC) $^ $(LDFLAGS) -o $@


.PHONY: clean
clean:
	-rm -f build/* cp210x-cfg
