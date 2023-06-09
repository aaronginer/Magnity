# Compiler and flags
CC = g++
CFLAGS = -std=c++14 -Wall -I/opt/homebrew/Cellar/sfml/2.5.1_2/include -I/opt/homebrew/Cellar/tgui/0.9.5/include -I/opt/homebrew/Cellar/nanoflann/1.4.3/include

# SFML libraries
SFML_LIBS = -L/opt/homebrew/Cellar/sfml/2.5.1_2/lib -lsfml-graphics -lsfml-window -lsfml-system
TGUI_LIBS = -L/opt/homebrew/Cellar/tgui/0.9.5/lib -ltgui
#NANOFLANN_LIBS = -L/opt/homebrew/Cellar/nanoflann/1.4.3/lib -nanoflann
LDFLAGS = $(SFML_LIBS) $(TGUI_LIBS) $(NANOFLANN_LIBS)

# Directories
SRC_DIR = ../src
OBJ_DIR = ./obj
BIN_DIR = ./bin

# Source files
SRCS = $(wildcard $(SRC_DIR)/*.cpp)
OBJS = $(patsubst $(SRC_DIR)/%.cpp,$(OBJ_DIR)/%.o,$(SRCS))

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
	$(CC) $(CFLAGS) $^ -o $@ $(LDFLAGS)

# Clean rule
clean:
	rm -rf $(OBJ_DIR)/* $(BIN_DIR)/Magnity

# Executable dependencies
$(BIN_DIR)/Magnity: $(OBJS)

# Source file dependencies
$(OBJ_DIR)/main.o: $(SRC_DIR)/main.cpp
$(OBJ_DIR)/Border.o: $(SRC_DIR)/Border.cpp $(SRC_DIR)/Border.h
$(OBJ_DIR)/Enemy.o: $(SRC_DIR)/Enemy.cpp $(SRC_DIR)/Enemy.h
$(OBJ_DIR)/Magnet.o: $(SRC_DIR)/Magnet.cpp $(SRC_DIR)/Magnet.h
$(OBJ_DIR)/Matrix.o: $(SRC_DIR)/Matrix.cpp $(SRC_DIR)/Matrix.h
$(OBJ_DIR)/Object.o: $(SRC_DIR)/Object.cpp $(SRC_DIR)/Object.h
$(OBJ_DIR)/Player.o: $(SRC_DIR)/Player.cpp $(SRC_DIR)/Player.h
$(OBJ_DIR)/PlayerArea.o: $(SRC_DIR)/PlayerArea.cpp $(SRC_DIR)/PlayerArea.h
$(OBJ_DIR)/PathInterpol.o: $(SRC_DIR)/PathInterpol.cpp $(SRC_DIR)/PathInterpol.h
$(OBJ_DIR)/VoronoiFracture.o: $(SRC_DIR)/VoronoiFracture.cpp $(SRC_DIR)/VoronoiFracture.h
$(OBJ_DIR)/Quaternion.o: $(SRC_DIR)/Quaternion.cpp $(SRC_DIR)/Quaternion.h
$(OBJ_DIR)/RigidBody.o: $(SRC_DIR)/RigidBody.cpp $(SRC_DIR)/RigidBody.h
$(OBJ_DIR)/Animation.o: $(SRC_DIR)/Animation.cpp $(SRC_DIR)/Animation.h
$(OBJ_DIR)/KDTreeVectorOfVectorsAdaptor.o: $(SRC_DIR)/KDTreeVectorOfVectorsAdaptor.cpp $(SRC_DIR)/KDTreeVectorOfVectorsAdaptor.h
$(OBJ_DIR)/KDTree.o: $(SRC_DIR)/KDTree.cpp $(SRC_DIR)/KDTree.h
