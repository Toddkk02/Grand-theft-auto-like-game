main:
	g++ main.cpp -o gta_traffic `pkg-config --cflags --libs sfml-graphics sfml-window sfml-system` -std=c++17
