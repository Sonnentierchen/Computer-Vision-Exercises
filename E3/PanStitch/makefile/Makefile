# Written by Dmitrij Schlesinger (2013),

# Tools -----------------------------------------------

CXX           = g++
LINK          = g++
DEL_FILE      = rm -f

# Flags (remove "-m64" everywhere for 32bit arch) -----

CXXFLAGS      = -m64 -pipe -O2 -Wall -W
LFLAGS        = -m64 

# Include path and libraries --------------------------

# OSX
INCPATH     += -I/opt/local/include
LIBS        += -L/opt/local/lib -lopencv_highgui -lopencv_videoio -lopencv_imgcodecs -lopencv_ml -lopencv_imgproc -lopencv_core -lm -lopencv_calib3d

# Linux
# INCPATH and additional library paths are standard since Ubuntu 12.04
# LIBS          = -lopencv_core -lopencv_highgui -lopencv_imgproc -lopencv_calib3d


#-------------------------------------------------

TARGET        = panst
SOURCES       = ../src/ps_main.cpp \
		../src/harris.cpp \
		../src/matching.cpp \
		../src/mean.cpp \
		../src/local_maxima.cpp \
		../src/saves.cpp \
		../src/render.cpp \
		../src/homographies.cpp
HEADERS       = ../src/ps.h
OBJECTS       = $(SOURCES:.cpp=.o)

all: $(TARGET)

$(TARGET):  $(OBJECTS)  
	$(LINK) $(LFLAGS) -o $(TARGET) $(OBJECTS) $(LIBS)

%.o : %.cpp $(HEADERS)
	$(CXX) -c $(CXXFLAGS) $(INCPATH) $< -o $@

clean:
	-$(DEL_FILE) $(OBJECTS)
	-$(DEL_FILE) *~ core *.core

distclean: clean
	-$(DEL_FILE) $(TARGET) 
