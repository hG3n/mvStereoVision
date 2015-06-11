# name of program
TARGETDIR = trgt
TARGETOBJDIR = trgt_obj
#compiler flags
CC = g++-4.9
CFLAGS  = -c -g -fPIC -Wall -std=c++11 -fdiagnostics-color=auto

#source directories
SRCDIR = src

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
INC_PATH = -I./inc/  -I/opt/mvIMPACT_acquire/ -I/usr/local/include/opencv2/

#library paths
LIB_PATH = -L/opt/mvIMPACT_acquire/lib/x86_64/ -L/usr/local/lib/

#libraries
OPENCV = `pkg-config opencv --libs`

LIBS = -lmvDeviceManager -lpthread $(OPENCV)

capture: directories $(OBJ)
	@$(CC) $(CFLAGS) $(INC_PATH) $(TARGETDIR)/capture.cpp -o $(TARGETOBJDIR)/capture.o
	@echo Compiled capture.cpp successfully!
	@$(CC) $(OBJ) $(TARGETOBJDIR)/capture.o -o $(BINDIR)/capture $(LIB_PATH) $(LIBS)
	@echo Linking complete!

liveView: directories $(OBJ)
	@$(CC) $(CFLAGS) $(INC_PATH) $(TARGETDIR)/liveView.cpp -o $(TARGETOBJDIR)/liveView.o
	@echo Compiled liveView.cpp successfully!
	@$(CC) $(OBJ) $(TARGETOBJDIR)/liveView.o -o $(BINDIR)/liveView $(LIB_PATH) $(LIBS)
	@echo Linking complete!

captureRectified: directories $(OBJ)
	@$(CC) $(CFLAGS) $(INC_PATH) $(TARGETDIR)/captureRectified.cpp -o $(TARGETOBJDIR)/captureRectified.o
	@echo Compiled captureRectified.cpp successfully!
	@$(CC) $(OBJ) $(TARGETOBJDIR)/captureRectified.o -o $(BINDIR)/captureRectified $(LIB_PATH) $(LIBS)
	@echo Linking complete!

captureDisparity: directories $(OBJ)
	@$(CC) $(CFLAGS) $(INC_PATH) $(TARGETDIR)/captureDisparity.cpp -o $(TARGETOBJDIR)/captureDisparity.o
	@$(CC) $(OBJ) $(TARGETOBJDIR)/captureDisparity.o -o $(BINDIR)/captureDisparity $(LIB_PATH) $(LIBS)
	@echo Linking complete!

distance: directories $(OBJ)
	@$(CC) $(CFLAGS) $(INC_PATH) $(TARGETDIR)/distanceMeasurement.cpp -o $(TARGETOBJDIR)/distanceMeasurement.o
	@$(CC) $(OBJ) $(TARGETOBJDIR)/distanceMeasurement.o -o $(BINDIR)/distanceMeasurement $(LIB_PATH) $(LIBS)
	@echo Linking complete!

tests: directories $(OBJ)
	@$(CC) $(CFLAGS) $(INC_PATH) $(TARGETDIR)/testsetting.cpp -o $(TARGETOBJDIR)/testsetting.o
	@$(CC) $(OBJ) $(TARGETOBJDIR)/testsetting.o -o $(BINDIR)/testsetting $(LIB_PATH) $(LIBS)
	@echo Linking complete!

calcDisparity: directories $(OBJ)
	@$(CC) $(CFLAGS) $(INC_PATH) $(TARGETDIR)/disparityTest.cpp -o $(TARGETOBJDIR)/disparityTest.o
	@$(CC) $(OBJ) $(TARGETOBJDIR)/disparityTest.o -o $(BINDIR)/disparityTest $(LIB_PATH) $(LIBS)
	@echo Linking complete!

epipolarLines: directories $(OBJ)
	@$(CC) $(CFLAGS) $(INC_PATH) $(TARGETDIR)/epipolarLines.cpp -o $(TARGETOBJDIR)/epipolarLines.o
	@echo Compiled epipolarLines.cpp successfully!
	@$(CC) $(OBJ) $(TARGETOBJDIR)/epipolarLines.o -o $(BINDIR)/epipolarLines $(LIB_PATH) $(LIBS)
	@echo Linking complete!

liveUndistortion: directories $(OBJ)
	@$(CC) $(CFLAGS) $(INC_PATH) $(TARGETDIR)/liveUndistortion.cpp -o $(TARGETOBJDIR)/liveUndistortion.o
	@echo Compiled liveUndistortion.cpp successfully!
	@$(CC) $(OBJ) $(TARGETOBJDIR)/liveUndistortion.o -o $(BINDIR)/liveUndistortion $(LIB_PATH) $(LIBS)
	@echo Linking complete!

calibrate-stereo: directories $(OBJ)
	@$(CC) $(CFLAGS) $(INC_PATH) $(TARGETDIR)/calibrate-stereo.cpp -o $(TARGETOBJDIR)/calibrate-stereo.o
	@echo Compiled calibrate-stereo.cpp successfully!
	@$(CC) $(OBJ) $(TARGETOBJDIR)/calibrate-stereo.o -o $(BINDIR)/calibrate-stereo $(LIB_PATH) $(LIBS)
	@echo Linking complete!

continousCapture: directories $(OBJ)
	@$(CC) $(CFLAGS) $(INC_PATH) $(TARGETDIR)/continousCapture.cpp -o $(TARGETOBJDIR)/continousCapture.o
	@echo Compiled continousCapture.cpp successfully!
	@$(CC) $(OBJ) $(TARGETOBJDIR)/continousCapture.o -o $(BINDIR)/continousCapture $(LIB_PATH) $(LIBS)
	@echo Linking complete!

continousCaptureRectified: directories $(OBJ)
	@$(CC) $(CFLAGS) $(INC_PATH) $(TARGETDIR)/continousCaptureRectified.cpp -o $(TARGETOBJDIR)/continousCaptureRectified.o
	@echo Compiled continousCaptureRectified.cpp successfully!
	@$(CC) $(OBJ) $(TARGETOBJDIR)/continousCaptureRectified.o -o $(BINDIR)/continousCaptureRectified $(LIB_PATH) $(LIBS)
	@echo Linking complete!

obstacle: directories $(OBJ)
	@$(CC) $(CFLAGS) $(INC_PATH) $(TARGETDIR)/obstacle.cpp -o $(TARGETOBJDIR)/obstacle.o
	@echo Compiled obstacle.cpp successfully!
	@$(CC) $(OBJ) $(TARGETOBJDIR)/obstacle.o -o $(BINDIR)/obstacle $(LIB_PATH) $(LIBS)
	@echo Linking complete!

test: directories $(OBJ)
	@$(CC) $(CFLAGS) $(INC_PATH) $(TARGETDIR)/test.cpp -o $(TARGETOBJDIR)/test.o
	@echo Compiled test.cpp successfully!
	@$(CC) $(OBJ) $(TARGETOBJDIR)/test.o -o $(BINDIR)/test $(LIB_PATH) $(LIBS)
	@echo Linking complete!

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


