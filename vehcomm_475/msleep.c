#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>

int main(int argc, char *argv[]) {

	int msec = 0;

	if(argv[1] == NULL ) {
		printf("Usage: %s <no. milliseconds to sleep>\n", argv[0]);
		exit(EXIT_FAILURE);
	}
	msec = atoi(argv[1]);

	usleep( (time_t)(msec*1000) );
	return 0;
}
