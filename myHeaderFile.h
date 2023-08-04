#ifndef MYHEADERFILE_H_
#define MYHEADERFILE_H_

// Macros
#define FREQ
#define PORT
#define LED1 GPIO_PORT_P1
#define LED2 GPIO_PORT_P2
#define RED GPIO_PIN0
#define GREEN GPIO_PIN1
#define BLUE GPIO_PIN2

// Prototypes
void initialize_LEDS(unsigned char port, unsigned short pins);
char rbg_cycle(int);

// Function Definitions

// Gets each character to be transmitted
void UART_sendChar(uint8_t c) {
    // wait until TXB is ready
    while (!UART_getInterruptStatus(EUSCI_A0_BASE, EUSCI_A_UART_TRANSMIT_INTERRUPT_FLAG));
    UART_transmitData(EUSCI_A0_BASE, c);
}

// Uses character function to send a string to Putty
void UART_sendString(const char* str) {
    while (*str) {
        UART_sendChar(*str++);
    }
}

// Receives a character via UART
uint8_t UART_receivedChar() {
    // wait until RXB is ready
    while (!UART_getInterruptStatus(EUSCI_A0_BASE, EUSCI_A_UART_RECEIVE_INTERRUPT_FLAG));
    return UART_receiveData(EUSCI_A0_BASE);
}

// Send Integer
void UART_sendInt(int number) {
    char buffer[20];  // Assuming the maximum number of digits is 20
    snprintf(buffer, sizeof(buffer), "%d", number);  // Convert the integer to a string
    UART_sendString(buffer);  // Send the string over UART
}

// Send Float
void UART_sendFloat( float number) {
    char buffer[20];  // Assuming the maximum number of digits is 20
    snprintf(buffer, sizeof(buffer), "%f", number);  // Convert the float to a string
    UART_sendString( buffer);  // Send the string over UART
}

void clearInputBuffer(uint32_t moduleInstance) {
    while (UART_getInterruptStatus(moduleInstance, EUSCI_A_UART_RECEIVE_INTERRUPT_FLAG)) {
        UART_receivedChar(moduleInstance);  // Clear received characters
    }
}


// Convert from inches to percent duty cycle
float inch2percent(float setPoint){
    float x; // setPoint in percent
    x = ((setPoint - 3)*(1 - 0)) / (36 - 3);
    return x;
}

// LED initialize
void initialize_LEDS(unsigned char port, unsigned short pins) {
    GPIO_setAsOutputPin(port, pins);
    GPIO_setOutputLowOnPin(port, pins);
}

// UART Function to send height data as a string
void sendHeightDataOverUART(uint32_t moduleInstance, float height) {
    char heightStr[20]; // Adjust the buffer size as needed
    snprintf(heightStr, sizeof(heightStr), "%.2f", height);
    UART_sendString( heightStr);
    UART_sendString( "\r\n"); // Add newline characters to indicate the end of the data.
}

// Function to control the LED
void flashLED(void) {
    GPIO_setOutputHighOnPin(LED1, RED);
    __delay_cycles(100000); // Delay for 0.5 seconds (adjust as needed)
    GPIO_setOutputLowOnPin(LED1, RED);
}
#endif /* MYHEADERFILE_H_ */
