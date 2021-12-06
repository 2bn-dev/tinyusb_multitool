
#include "pico/stdlib.h"
#include "pico/time.h"

void fatal_error_infinite_loop(){
	printf("\nExecution Stalled\n");
	while(true){
		_DBG("FATAL ERROR STALL!");
		sleep_ms(1000);
	}
}

void fatal_error(const char * detail){
	printf("\n\nFATAL: %s!\n\n\n", detail);
	fatal_error_infinite_loop();
}

void warning(const char * detail){
	printf("\n\nWARNING: %s!\n\n\n", detail);
}

int main(){
	return 0;
}


