CC = g++
CFLAGS = -Wall -Wextra -std=c++17

SRCDIR = src
OBJDIR = obj

SRCS = $(wildcard $(SRCDIR)/*.cpp)
OBJS = $(patsubst $(SRCDIR)/%.cpp,$(OBJDIR)/%.o,$(SRCS))
EXECUTABLE = nodes

.PHONY: all clean debug

all: $(EXECUTABLE)

$(EXECUTABLE): $(OBJS)
        $(CC) $(CFLAGS) $^ -o $@

$(OBJDIR)/%.o: $(SRCDIR)/%.cpp
        @mkdir -p $(@D)
        $(CC) $(CFLAGS) -c $< -o $@

debug: CFLAGS += -g
debug: CFLAGS += -O0
debug: CFLAGS += -DDEBUG
debug: $(EXECUTABLE)

clean:
        rm -rf $(OBJDIR) $(EXECUTABLE)
