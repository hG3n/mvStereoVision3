# name of program
TARGETDIR = trgt
TARGETOBJDIR = trgt_obj
#compiler flags
CC = g++
CFLAGS  = -c -g -fPIC -Wall -std=c++11

#source directories
SRCDIR = ./../src

#binary directories
OBJDIR = obj
BINDIR = bin

#soruce files
SRC = $(wildcard $(SRCDIR)/*.cpp)

#obj files
OBJ = $(SRC:$(SRCDIR)/%.cpp=$(OBJDIR)/%.o)

#commands
MKDIR_P = mkdir -p
RM = rm -rf

#include path
INC_PATH = -I./../inc/  -I/opt/mvIMPACT_acquire/ -I/usr/local/include/opencv2/

#library paths
LIB_PATH = -L/opt/mvIMPACT_acquire/lib/x86_64/ -L/usr/local/lib/

#libraries
OPENCV = `pkg-config opencv --libs`

LIBS = -lmvDeviceManager -lpthread $(OPENCV)


images: directories $(OBJ)
	$(CC) $(CFLAGS) $(INC_PATH) ./images.cpp -o $(TARGETOBJDIR)/images.o
	echo Compiled images.cpp successfully!
	$(CC) $(OBJ) $(TARGETOBJDIR)/images.o -o $(BINDIR)/images $(LIB_PATH) $(LIBS)
	echo Linking complete!

disparity: directories $(OBJ)
	$(CC) $(CFLAGS) $(INC_PATH) ./disparity.cpp -o $(TARGETOBJDIR)/disparity.o
	echo Compiled disparity.cpp successfully!
	$(CC) $(OBJ) $(TARGETOBJDIR)/disparity.o -o $(BINDIR)/disparity $(LIB_PATH) $(LIBS)
	echo Linking complete!

directories:
	${MKDIR_P} ${OBJDIR}
	${MKDIR_P} ${BINDIR}
	${MKDIR_P} ${TARGETOBJDIR}


$(OBJ): $(OBJDIR)/%.o :$(SRCDIR)/%.cpp
	@$(CC) $(CFLAGS) $(INC_PATH) $< -o $@
	@echo Compiled $< $ successfully!

clean:
	$(RM) $(OBJDIR) $(BINDIR) $(TARGETOBJDIR)

#To compile mvIMPACTacquire SDK with c++11 find the file:
#/opt/mvIMPACT_acquire/mvPropHandling/include/mvPropHandlingDatatypes.h
#and change the line (near line 27)
#   elif defined(linux)
#to
#    elif defined(__linux__)


