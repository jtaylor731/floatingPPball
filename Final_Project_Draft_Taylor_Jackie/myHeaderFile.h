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
    sprintf(buffer, "%d", number);  // Convert the integer to a string
    UART_sendString(buffer);  // Send the string over UART
}

void clearInputBuffer(void) {
    while (UART_getInterruptStatus(EUSCI_A0_BASE, EUSCI_A_UART_RECEIVE_INTERRUPT_FLAG)) {
        UART_receivedChar();  // Clear received characters
    }
}

// LED initialize
void initialize_LEDS(unsigned char port, unsigned short pins) {
    GPIO_setAsOutputPin(port, pins);
    GPIO_setOutputLowOnPin(port, pins);
}

#endif /* MYHEADERFILE_H_ */
