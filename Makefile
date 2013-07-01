all: kiwiread

kiwiread: kiwiread.c bmp.c
	gcc -Wall -g -o kiwiread kiwiread.c bmp.c -lm

