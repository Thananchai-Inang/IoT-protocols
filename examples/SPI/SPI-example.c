#define F_CPU 16000000UL

#include <avr/io.h>
#include <util/delay.h>

#define MAX_DEVICES 1 // Change this based on the number of MAX7219 devices you have in series
#define CS_PIN PB2     // Define the Chip Select (CS) pin
#define CLK_PIN PB5    // Define the SPI clock pin
#define DATA_PIN PB3   // Define the SPI data pin

void spiInit();
void spiSend(unsigned char data);
void max7219Init();
void max7219Send(unsigned char reg, unsigned char data);
void displayChangeIntensity();
void displayPattern();

int intensity = 1;

int main(void) {
	spiInit();
	max7219Init();
	displayPattern();
	while (1) {
		displayChangeIntensity();
		_delay_ms(1000);

	}

	return 0;
}

void spiInit(){
	// Set CS, CLK, and DATA pins as outputs
	DDRB |= (1 << CS_PIN) | (1 << CLK_PIN) | (1 << DATA_PIN);
	// Set up SPI
	SPCR |= (1 << SPE) | (1 << MSTR) | (1 << SPR0);  // Enable SPI, Master mode, Clock = F_CPU/16
	SPSR |= (0 << SPI2X); //
}

void spiSend(unsigned char data) {
	SPDR = data;
	while (!(SPSR & (1 << SPIF)));
}

void max7219Send(unsigned char reg, unsigned char data) {
	PORTB &= ~(1 << CS_PIN);  // CS low

	spiSend(reg);
	spiSend(data);

	PORTB |= (1 << CS_PIN);  // CS high
}

void max7219Init() {
	max7219Send(0x09, 0x00);  // Decode mode: No decode for digits 0-7
	max7219Send(0x0A, 0x0F);  // Intensity: Set to maximum
	max7219Send(0x0B, 0x07);  // Scan limit: Display digits 0-7
	max7219Send(0x0C, 0x01);  // Shutdown mode: Normal operation
}



void displayChangeIntensity() {
	if(intensity == 1){intensity=15;}
	else{intensity=1;}
	max7219Send(0x0A, intensity);
}

void displayPattern() {
	// Define a simple smiley face pattern
	unsigned char smiley[8] = {
		0b00111100,
		0b01000010,
		0b10100101,
		0b10000001,
		0b10100101,
		0b10011001,
		0b01000010,
		0b00111100
	};

	// Display the pattern on the matrix
	for (int i = 0; i < 8; i++) {
		max7219Send(i + 1, smiley[i]);  // Start from register 1
	}
}
