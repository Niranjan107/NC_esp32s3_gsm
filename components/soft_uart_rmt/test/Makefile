CC      = gcc
CFLAGS  = -std=c11 -Wall -Wextra -Werror -O0 -g -I.. -I../include
TARGET  = test_rmt_decoder
SRCS    = test_rmt_decoder.c ../rmt_decoder.c

all: $(TARGET)

$(TARGET): $(SRCS)
	$(CC) $(CFLAGS) -o $(TARGET) $(SRCS)

test: $(TARGET)
	./$(TARGET)

clean:
	rm -f $(TARGET) $(TARGET).exe

.PHONY: all test clean
