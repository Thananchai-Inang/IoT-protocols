//I2C
#define F_CPU 16000000UL  // Assuming a 16MHz clock frequency
#define I2C_BITRATE 100000 // I2C bus speed in Hz
#define GY30_ADDRESS 0x23 // Replace with the correct address for GY-30
#define DS3231_ADDRESS 0x68 // Replace with the correct address for DS3231

#include <avr/io.h>
#include <util/twi.h>
#include <util/delay.h>
#include <stdio.h>
#include <stdint.h>

//SPI
#define F_CPU 16000000UL

#include <avr/io.h>
#include <util/delay.h>

#define MAX_DEVICES 1 // Change this based on the number of MAX7219 devices you have in series
#define CS_PIN PB2     // Define the Chip Select (CS) pin
#define CLK_PIN PB5    // Define the SPI clock pin
#define DATA_PIN PB3   // Define the SPI data pin

int intensity = 1;

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

void IntensityHigh() {
  intensity = 15;
  max7219Send(0x0A, intensity);
}

void IntensityLow() {
  intensity = 1;
  max7219Send(0x0A, intensity);
}

void displayChangeIntensity() {
	if(intensity == 1){intensity=15;}
	else{intensity=1;}
	max7219Send(0x0A, intensity);
}

void displayPattern() {
	// Define a simple smiley face pattern
	unsigned char smiley[8] = {
		0b00000000,
		0b01100110,
		0b11111111,
		0b11111111,
		0b01111110,
		0b00111100,
		0b00011000,
		0b00000000
	};

	// Display the pattern on the matrix
	for (int i = 0; i < 8; i++) {
		max7219Send(i + 1, smiley[i]);  // Start from register 1
	}
}


void initSerial() {

	// Set baud rate 9600
	UBRR0H = 0x00;
	UBRR0L = 0x67;

	// Enable transmitter and receiver
	UCSR0B = (1 << TXEN0) | (1 << RXEN0);

	// Set frame format: 8 data bits, 1 stop bit
	UCSR0C = (1 << UCSZ01) | (1 << UCSZ00);
}

void sendSerial(char sbuf[],int y){
	char *pp = &sbuf[0];
	while(y) {
		while(!(UCSR0A & 0b00100000));
		UDR0 = *pp++;
		y--;
	}
}

void initI2C() {
	// Set prescaler to 1
	TWSR = 0;
	
	// Calculate bit rate
	TWBR = ((F_CPU / I2C_BITRATE) - 16) / 2;

	// Enable TWI, generate ACK, enable TWI interrupt
	TWCR = (1 << TWEN) | (1 << TWEA);
}

void startI2C() {
	// Send start condition
	TWCR = (1 << TWSTA) | (1 << TWINT) | (1 << TWEN);
	
	// Wait until start condition is sent
	while (!(TWCR & (1 << TWINT)));
	
	// Check status code (this should be 0x08 (START) or 0x10 (re-START))
	if(((TWSR & 0xF8) != 0x08) && ((TWSR & 0xF8) != 0x10)) {
		char error_msg[32];
		int size_emsg;
		size_emsg = sprintf(error_msg,"Error startI2C %x \n",(TWSR & 0xF8));
		sendSerial(error_msg,size_emsg);
	}
}

void stopI2C() {
	// Send stop condition
	TWCR = (1 << TWSTO) | (1 << TWINT) | (1 << TWEN);
}

void writeI2C(uint8_t data) {
	// Load data into TWI data register
	TWDR = data;

	// Clear the TWINT bit to start data transmission
	TWCR = (1 << TWINT) | (1 << TWEN);

	// Wait until transmission is complete
	while (!(TWCR & (1 << TWINT)));
	
	// Check status code (this should be 0x20 (sent SLA+W, ack received),0x40 (sent SLA+R, ack received), or 0x28 (sent data, received))
	if(((TWSR & 0xF8) != 0x18) && ((TWSR & 0xF8) != 0x28) && ((TWSR & 0xF8) != 0x40)) {
		char error_msg[32];
		int size_emsg;
		size_emsg = sprintf(error_msg,"Error writeI2C %x \n",(TWSR & 0xF8));
		sendSerial(error_msg,size_emsg);
	}
}

uint8_t readI2C(uint8_t ack) {
	// Enable or disable ACK bit based on ack parameter
	if (ack) {
		TWCR = (1 << TWINT) | (1 << TWEN) | (1 << TWEA);
		} else {
		TWCR = (1 << TWINT) | (1 << TWEN);
	}

	// Wait until data is received
	while (!(TWCR & (1 << TWINT)));
	
	// Check status code (this should be 0x50 (sent data, ack sent),0x58 (sent data, N-ack sent)
	if(((TWSR & 0xF8) != 0x58) && ((TWSR & 0xF8) != 0x50)) {
		char error_msg[32];
		int size_emsg;
		size_emsg = sprintf(error_msg,"Error readI2C %x \n",(TWSR & 0xF8));
		sendSerial(error_msg,size_emsg);
	}

	// Return received data
	return TWDR;
}

void initDS3231() {
	// Assuming you want to set the initial time to 12:00:00
	startI2C();
	writeI2C(DS3231_ADDRESS << 1); // Write address
	writeI2C(0x00); // Set register pointer to 0x00 (Seconds)
	writeI2C(0x00); // Seconds
	writeI2C(0x00); // Minutes
	writeI2C(0x12); // Hours (12-hour format)
	// Set other relevant registers as needed
	stopI2C();
}

void writeDS3231(uint8_t reg, uint8_t data) {
	startI2C();
	writeI2C(DS3231_ADDRESS << 1); // Write address
	writeI2C(reg); // Set register pointer
	writeI2C(data); // Write data
	stopI2C();
}

uint8_t readDS3231(uint8_t reg) {
	startI2C();
	writeI2C(DS3231_ADDRESS << 1); // Write address
	writeI2C(reg); // Set register pointer
	stopI2C();

	startI2C();
	writeI2C((DS3231_ADDRESS << 1) | 1); // Read address
	uint8_t data = readI2C(0); // Read data with NACK
	stopI2C();

	return data;
}


uint16_t readGY30() {
	startI2C();
	writeI2C(GY30_ADDRESS << 1); // Write address
	writeI2C(0x10); // Command to start measurement
	stopI2C();

	// Wait for measurement (adjust delay based on your module's specifications)
	_delay_ms(180);

	startI2C();
	writeI2C((GY30_ADDRESS << 1) | 1); // Read address
	uint8_t msb = readI2C(1); // Read MSB with ACK
	uint8_t lsb = readI2C(0); // Read LSB with NACK
	stopI2C();

	// Combine MSB and LSB to get the final result
	return (msb << 8) | lsb;
}



int main() {
	char print_time_buff[32], print_intensity_buff[32];
	int time_string_size,intensity_string_size ;
	
  //I2C init
	initSerial();
	initI2C();
	initDS3231();
  //SPI init
  spiInit();
	max7219Init();
	displayPattern();

	while (1) {
		uint16_t lightIntensity = readGY30();
		//uint8_t hour = readDS3231(0x02);
		//uint8_t minute = readDS3231(0x01);
		//uint8_t second = readDS3231(0x00);
		
		
		// uint8_t decimalHour = ((hour & 0xF0) >> 4) * 10 + (hour & 0x0F);
		// uint8_t decimalMinute = ((minute & 0xF0) >> 4) * 10 + (minute & 0x0F);
		// uint8_t decimalSecond = ((second & 0xF0) >> 4) * 10 + (second & 0x0F);

		
		
		// time_string_size = sprintf(print_time_buff,"Current Time: %02d:%02d:%02d >> ", decimalHour, decimalMinute, decimalSecond);
		intensity_string_size = sprintf(print_intensity_buff," Light Intensity: %u\n", lightIntensity);
		
		// sendSerial(print_time_buff,time_string_size);
		sendSerial(print_intensity_buff,intensity_string_size);
    
    if (lightIntensity > 1000) {
      IntensityHigh();
		  _delay_ms(100);
    } else {
      IntensityLow();
    }

		// Wait for a moment before the next iteration (adjust delay based on your needs)
		_delay_ms(100);
	}

	return 0;
}


