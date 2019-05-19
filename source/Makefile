OBJS = z80bus.o 
TARGET = z80
LDFLAGS = -lbcm2835 

.SUFFIXES: .c .o

%.o: %.c
	$(CC) -c -o $@ $< -D_MAIN

all: $(TARGET)

$(TARGET): $(OBJS)
	$(CC) -o $@ -g $(OBJS) $(LDFLAGS)

clean:
	rm -f $(OBJS) $(TARGET) m
