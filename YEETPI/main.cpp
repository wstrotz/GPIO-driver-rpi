#include <cstdio>
#include <time.h>
#include "UNR_GPIO_BCM2711.h"

void delay(int NOS) {
	int ms = 1000 * NOS;

	clock_t st = clock();
	while (clock() < st + ms);
}

int main() {
	setup();
	delay(10);

	int LEDPIN = 2;
	int BUTTON = 4;
	setup_gpio(LEDPIN, OUTPUT, PUD_OFF);
	delay(10);
	setup_gpio(BUTTON, INPUT, PUD_UP);
	delay(10);

	int state = 0;

	while (1) {
		if (input_gpio(BUTTON) == HIGH) {
			if (state == 0)
				state += 1;
			else
				state = 0;
			delay(1000);
		}
		if (state == 0)
			output_gpio(LEDPIN, HIGH);
		else
			output_gpio(LEDPIN, LOW);
				
			
	}
	/* Blink LED every second
	for (int i = 0; i < 100; i++) {
		output_gpio(LEDPIN, HIGH);
		delay(1000);
		output_gpio(LEDPIN, LOW);
		delay(1000);
	} 
	*/
	cleanup();

	/* Old main
	char result;
	printf("You enter a dark dark forest, the path turns to the right, what direction do you take? (L/R)");
	scanf("%c", &result);
	if ((result == 'L') || (result == 'l'))
	{
		printf("You are dumb\n\n\nTHE END\n\n\n");
	}
	else if ((result == 'R') || (result == 'r'))
	{
		printf("You know how to read directions...\n\n\n");
		printf("The path continues, do you follow it? (Y/N)");
		scanf(" %c", &result);
		if ((result == 'Y') || (result == 'y')) {
			printf("You continue to follow the path\n\n\nYou are mauled by a bear\n\n\nThe End\n\n\n");

		}
		else if ((result == 'N') || (result == 'n'))
			printf("You don't move down the path\n\n\nA hammer falls from the sky and you are struck in the head and die to bluntforce trauma to the head\n\n\nThe End\n\n\n");
	}
	*/
	return 0;
}