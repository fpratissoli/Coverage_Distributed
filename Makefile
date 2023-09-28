#.DEFAULT_GOAL:= generate_files set the default target, by default it is always the first one


#in PHONY sono elencati i comandi indipendentemente dai nomi dei file nella directory -> se faccio >>make clean, ed esiste un file chiamato "clean.C" viene eseguito il comando nel Makefile e ignorato il file

.PHONY: build_main clean compile_main compile_main_in_build generate_build_directory

build_main: generate_build_directory compile_main_in_build send_notification
	@echo "project compiled in build directory"

compile_main:
	g++ -c main.cpp
	g++ main.o -o main_exe -lsfml-graphics -lsfml-window -lsfml-system

compile_main_in_build:
	g++ -c main.cpp -o build/main.o
	g++ build/main.o -o build/main_exe -lsfml-graphics -lsfml-window -lsfml-system

clean:
	@echo "cleaning up build folder ..."
	rm -r build/*

generate_build_directory:
	mkdir -p build
	@echo "Build folder generated"

send_notification:
	notify-send "Hey pal, build has been completed!"
