all: exercise

exercise: 6dicp.cc mini.o ann_0.2/lib/libANN.a newmat/libnewmat.a
	g++ -g -o 6dicp 6dicp.cc mini.o -lGL -lGLU -lglut -I ann_0.2/include -Lann_0.2/lib/ -lANN -I newmat/ -L newmat/ -lnewmat

ann_0.2/lib/libANN.a:
	cd ann_0.2 && make

newmat/libnewmat.a:
	cd newmat && make

mini.o: mini.cc
	g++ -g -o mini.o -c mini.cc -I newmat/ 

clean:
	rm -f *.o; rm *.log; rm *.backup; rm *~; rm 6dicp
