all: kiwiread

kiwiread: kiwiread.c bmp.c
	gcc -g -o kiwiread kiwiread.c bmp.c -lm

