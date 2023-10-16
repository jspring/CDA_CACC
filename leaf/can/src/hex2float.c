#include <stdio.h>
#include <stdlib.h>

int main(int argc, char *argv[]) {
	float mult = 1;

	if(argv[2] != NULL)
		mult = strtof(argv[2], NULL);
	printf("Hex: %#lx Decimal: %d %d %lf\n", 
		strtol(argv[1], NULL, 0), 
		strtol(argv[1], NULL, 0), 
		(short)(strtol(argv[1], NULL, 0)), 
		mult * strtol(argv[1], NULL, 0));
}
