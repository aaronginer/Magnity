# Compiler and flags
CC = g++
CFLAGS = -std=c++14 -Wall # -Werror -pedantic

# SFML libraries
SFML_LIBS = -L./opt/homebrew/Cellar/sfml/2.5.1_2/lib -lsfml-graphics -lsfml-window -lsfml-system

# Directories
SRC_DIR = ./src
OBJ_DIR = ./obj
BIN_DIR = ./bin

# Source files
SRCS = $(wildcard $(SRC_DIR)/main.cpp $(SRC_DIR)/*.cpp)
OBJS = $(SRCS:$(SRC_DIR)/%.cpp=$(OBJ_DIR)/%.o)

# Target executable
TARGET = $(BIN_DIR)/Magnity

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
	$(CC) $(CFLAGS) $^ -o $@ $(SFML_LIBS)

# Clean rule
clean:
	rm -rf $(OBJ_DIR)/* $(BIN_DIR)/Magnity

# Executable dependencies
$(BIN_DIR)/Magnity: $(OBJ_DIR)/main.o $(OBJ_DIR)/Animation.o $(OBJ_DIR)/Border.o $(OBJ_DIR)/Contact.o $(OBJ_DIR)/Enemy.o $(OBJ_DIR)/Magnet.o $(OBJ_DIR)/Matrix.o $(OBJ_DIR)/Object.o $(OBJ_DIR)/Player.o $(OBJ_DIR)/PlayerArea.o $(OBJ_DIR)/Quaternion.o $(OBJ_DIR)/RigidBody.o $(OBJ_DIR)/PathInterpol.o

# Source file dependencies
$(OBJ_DIR)/main.o: $(SRC_DIR)/main.cpp
	$(CC) $(CFLAGS) -c $< -o $@

$(OBJ_DIR)/Animation.o: $(SRC_DIR)/Animation.cpp $(SRC_DIR)/Animation.h
	$(CC) $(CFLAGS) -c $< -o $@

$(OBJ_DIR)/Border.o: $(SRC_DIR)/Border.cpp $(SRC_DIR)/Border.h
	$(CC) $(CFLAGS) -c $< -o $@

$(OBJ_DIR)/Contact.o: $(SRC_DIR)/Contact.cpp $(SRC_DIR)/Contact.h
	$(CC) $(CFLAGS) -c $< -o $@

$(OBJ_DIR)/Enemy.o: $(SRC_DIR)/Enemy.cpp $(SRC_DIR)/Enemy.h
	$(CC) $(CFLAGS) -c $< -o $@

$(OBJ_DIR)/Magnet.o: $(SRC_DIR)/Magnet.cpp $(SRC_DIR)/Magnet.h
	$(CC) $(CFLAGS) -c $< -o $@

$(OBJ_DIR)/Matrix.o: $(SRC_DIR)/Matrix.cpp $(SRC_DIR)/Matrix.h
	$(CC) $(CFLAGS) -c $< -o $@

$(OBJ_DIR)/Object.o: $(SRC_DIR)/Object.cpp $(SRC_DIR)/Object.h
	$(CC) $(CFLAGS) -c $< -o $@

$(OBJ_DIR)/Player.o: $(SRC_DIR)/Player.cpp $(SRC_DIR)/Player.h
	$(CC) $(CFLAGS) -c $< -o $@

$(OBJ_DIR)/PlayerArea.o: $(SRC_DIR)/PlayerArea.cpp $(SRC_DIR)/PlayerArea.h
	$(CC) $(CFLAGS) -c $< -o $@

$(OBJ_DIR)/PathInterpol.o: $(SRC_DIR)/PathInterpol.cpp $(SRC_DIR)/PathInterpol.h
	$(CC) $(CFLAGS) -c $< -o $@

$(OBJ_DIR)/Quaternion.o: $(SRC_DIR)/

