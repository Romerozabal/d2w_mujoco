
#LINUX
COMMON=-O2 -I../include -L../bin -mavx -pthread -Wl,-rpath,'$$ORIGIN'
LIBS = -lmujoco200 -lGL -lm -lglew ../bin/libglfw.so.3 -I/usr/include/eigen3/
CC = g++

ROOT = project

all:
	$(CC) $(COMMON) main.c $(LIBS) -o ../bin/$(ROOT)

main.o:
	$(CC) $(COMMON) -c main.c

clean:
	rm *.o ../bin/$(ROOT)
