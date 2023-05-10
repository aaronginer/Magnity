# Compiler and flags
CC = g++
CFLAGS = -std=c++17 -Wall -g# -Werror -pedantic

# SFML libraries
SFML_LIBS = -lsfml-graphics -lsfml-window -lsfml-system

# SFML libraries
TGUI_LIBS = -ltgui

# Directories
SRC_DIR = src
OBJ_DIR = obj
BIN_DIR = bin

# Source files
SRCS = $(wildcard $(SRC_DIR)/*.cpp)
OBJS = $(SRCS:$(SRC_DIR)/%.cpp=$(OBJ_DIR)/%.o)

# Target executable
TARGET = $(BIN_DIR)/magnity

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
	rm -rf $(OBJ_DIR)/* $(BIN_DIR)/magnity
