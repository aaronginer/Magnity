# Compiler and flags
CC = g++
CFLAGS = -std=c++17 -Wall -I../winlib/SFML/SFML-2.5.1/include -I../winlib/TGUI/TGUI-0.9/include

# SFML libraries
SFML_LIBS = -lsfml-graphics -lsfml-window -lsfml-system

# TGUI libraries
TGUI_LIBS = -ltgui

# Directories
SRC_DIR = ../src
OBJ_DIR = ../obj
BIN_DIR = ../bin

# Source files
SRCS = $(wildcard $(SRC_DIR)/*.cpp) $(wildcard $(SRC_DIR)/*/*.cpp)
OBJS = $(patsubst $(SRC_DIR)/%.cpp,$(OBJ_DIR)/%.o,$(SRCS))

# Target executable
TARGET = $(BIN_DIR)/magnity.exe

# Phony targets
.PHONY: all clean

# Default target
all: $(TARGET)

# Rule for building object files
$(OBJ_DIR)/%.o: $(SRC_DIR)/%.cpp
	@mkdir -p $(@D)
	$(CC) $(CFLAGS) -c $< -o $@

# Rule for building the executable
$(TARGET): $(OBJS)
	@mkdir -p $(@D)
	$(CC) $(CFLAGS) $^ -o $@ $(SFML_LIBS) $(TGUI_LIBS)

# Clean rule
clean:
	rm -rf $(OBJ_DIR)/* $(BIN_DIR)/magnity.exe
