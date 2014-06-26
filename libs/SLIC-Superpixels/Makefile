test_slic: slic.cpp test_slic.cpp
	g++ slic.cpp test_slic.cpp -o test_slic `pkg-config --cflags --libs opencv`

seg_dog.png: dog.png test_slic
	./test_slic dog.png 400 40 seg_dog.png
