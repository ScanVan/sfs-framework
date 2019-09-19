directory:
	mkdir -p bin
	mkdir -p obj

source:
	g++ src/GaussianPyramid.cpp -o obj/GaussianPyramid.o -c -O3 -std=c++14 -flto
	g++ src/Stochastic.cpp -o obj/Stochastic.o -c -O3 -std=c++14 -flto
	g++ src/OpticalFlow.cpp -o obj/OpticalFlow.o -c -O3 -std=c++14 -flto

all:directory source
	gcc-ar rcs bin/libflow.a obj/*.o

clean:
	rm -f obj/*.o
	rm -f bin/*.a
