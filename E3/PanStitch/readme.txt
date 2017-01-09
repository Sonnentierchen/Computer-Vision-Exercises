Panorama Stitching Code for the third Exercise "Computer Vision" WS2016/2015

________________________________________________________________________________

Content:

images, inf, land_col, land_normal, land_small_overlap -- examples: images + 
results

makefile -- contains only the Makefile for the Linux/OSX-build, go to the 
directory and type "make"

src -- sources (*.cpp, *.h)

VisSt2010 -- Microsoft Visual Studio 2010 Solution (Windows)

panst.pro -- qmake project, can be used from the console by typing "qmake" 
followed by "make", can be loaded into QtCreator (both Windows and Linux)
________________________________________________________________________________

All the stuff was tested under following configurations:

1) LinuxMint 17.1 Mate 1.8.1, GCC 4.8.2, 64 bit, OpenCV 2.4.8,
a) Makefile, b) QtCreator 3.0.1 with Qt 5.2.1 

2) macOS Sierra, 10.2.1, Apple LLVM version 8.0.0 (clang-800.0.42.1), Makefile,
OpenCV 3.1.0

3) Windows 10, MS Visual Studio Community 2015, OpenCV 2.4.13
Quite a lot of steps were necessary: 
compile OpenCV from sources using CMake, import VS2010 solution into VS2015, 
set thousands of paths, etc. But after all it seemed working :-) .
