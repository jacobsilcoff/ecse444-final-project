SRCS=Client.cpp
TARGET=client
OBJS=${SRCS:.cpp=.o}
CC=g++
CPPFLAGS=-std=c++11 -stdlib=libc++ -Wno-c++98-compat

all: compile $(TARGET)

compile: $(SRCS)
	$(CC) $(CFLAGS) $(CPPFLAGS) -c $^

$(TARGET): $(OBJS)
	$(CC) $(CFLAGS) $(CPPFLAGS) -o $@ $^

clean: 
	rm $(OBJS)