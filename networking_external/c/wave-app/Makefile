
CFLAGS += -DPLATFORM_UNEX -g -Wall -O0
LDLIBS += -lm -lpcap -lpthread

.phony: all clean

all: wave-app

clean: 
	rm -f wave-app radiotap.o transceiver.o wave-app.o utils.o
 
wave-app: radiotap.o transceiver.o wave-app.o utils.o


