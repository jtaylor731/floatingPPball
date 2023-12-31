/*FINAL PROJECT:
 * PID Demo:
 * Ping-pong ball floating to set point
 * Fan uses PWM
 * Set point is used using PUTTY
 * Ultrasonic sensor provides feedback
 *----------------------------------------------------------------
 *----------------------------------------------------------------
 *----------------------------------------------------------------
 */
#include <stdio.h>
#include "msp.h"
#include "driverlib.h"
#include "myHeaderFile.h"

///////////////////// SETUP ///////////////////////
// DEFINE
#define LED1 GPIO_PORT_P1
#define LED2 GPIO_PORT_P2
#define RED  GPIO_PIN0
#define GREEN  GPIO_PIN1
#define BLUE  GPIO_PIN2


// GLOBAL VARIABLES
const uint16_t ta2ccr0 = 3000;
volatile float duty_cycle = 0; // subject to change based on interrupt
volatile float pot_v;
volatile float ADC_result;
volatile float test;


unsigned short sw1_cur = 1; // current state of click
unsigned short sw1_old = 1; // previous state of click
unsigned short onoff1 = 0; // checks if SW1 'on' or 'off'

volatile uint16_t setPoint = 0; // Set point
volatile bool readingUART = false; // monitors if UART is reading or writing

// TIMER
const Timer_A_UpModeConfig upConfig =
{
 TIMER_A_CLOCKSOURCE_ACLK,          // ACLK Clock Source = 128 kHz
 TIMER_A_CLOCKSOURCE_DIVIDER_4,     // 128 Hz / 4 = 32000 Hz (CCR0 counts up this many times per second)
 3200,                              // This is CCR0: used to set the timer period: 3200 / 32000 = 0.1s period = 10 Hz
 TIMER_A_TAIE_INTERRUPT_DISABLE,
 TIMER_A_CCIE_CCR0_INTERRUPT_ENABLE,
 TIMER_A_DO_CLEAR
};


// PWM
Timer_A_PWMConfig pwmConfig =
{
 TIMER_A_CLOCKSOURCE_SMCLK,          // Frequency = 3 MHz
 TIMER_A_CLOCKSOURCE_DIVIDER_1,      // 3 MHz / 1 = 3 MHz
 ta2ccr0,                               // TA2CCR0: 3000/ (1) / (3,000,000 Hz) = (0.001 s ) PWM period = 1000
 TIMER_A_CAPTURECOMPARE_REGISTER_1 , // TA2CCR1 Resister - pin 2.4 T0.1
 TIMER_A_OUTPUTMODE_RESET_SET,
 0                                   // TA2CCR1:inital value (duty cycle)
};

// UART CONFIGURATION 57600 bps
const eUSCI_UART_Config UART_init =
{
 EUSCI_A_UART_CLOCKSOURCE_SMCLK,
 3,
 4,
 2,
 EUSCI_A_UART_NO_PARITY,
 EUSCI_A_UART_LSB_FIRST,
 EUSCI_A_UART_ONE_STOP_BIT,
 EUSCI_A_UART_MODE,
 EUSCI_A_UART_OVERSAMPLING_BAUDRATE_GENERATION
};

///////////////////// MAIN CODE ///////////////////////
void main(void)
{
    // Set up LEDs
    initialize_LEDS(LED2,RED|GREEN|BLUE);

    //for timers
    WDT_A_holdTimer(); // stop  s WDT
    FPU_enableModule();
    CS_setDCOFrequency(3E+6); // 1113 MHz
    CS_initClockSignal(CS_SMCLK, CS_DCOCLK_SELECT, CS_CLOCK_DIVIDER_1);
    CS_setReferenceOscillatorFrequency(CS_REFO_128KHZ);
    CS_initClockSignal(CS_ACLK, CS_REFOCLK_SELECT,CS_CLOCK_DIVIDER_1);

    // ***** Set Inputs & Outputs ***** //
    GPIO_setAsInputPinWithPullUpResistor(GPIO_PORT_P1,GPIO_PIN1); // S1 as an input
    GPIO_setAsInputPinWithPullUpResistor(GPIO_PORT_P1,GPIO_PIN4); // S2 as an input

    GPIO_setAsPeripheralModuleFunctionInputPin(GPIO_PORT_P5,GPIO_PIN5,GPIO_TERTIARY_MODULE_FUNCTION); // ADC Pin P5.5 - for sensor feedback
    GPIO_setAsPeripheralModuleFunctionOutputPin(GPIO_PORT_P2,GPIO_PIN4,GPIO_PRIMARY_MODULE_FUNCTION); // P2.4 --> Out to fan

    // ***** UART SET UP ***** // (from Lab 4- Putty and UART)
    GPIO_setAsPeripheralModuleFunctionInputPin(GPIO_PORT_P1, GPIO_PIN2 | GPIO_PIN3, GPIO_PRIMARY_MODULE_FUNCTION);// GPIO and UART TX/RX
    UART_initModule(EUSCI_A0_BASE, &UART_init); //( module , configuration)
    UART_enableModule(EUSCI_A0_BASE);// Initialize UART 0
    UART_enableInterrupt(EUSCI_A0_BASE, EUSCI_A_UART_RECEIVE_INTERRUPT);    // Enable UART receive interrupts
    Interrupt_enableInterrupt(INT_EUSCIA0);
    Interrupt_enableMaster();    // Enable Interrupt at NVIC

    // Disable all Interrupts
    Interrupt_disableMaster();

    //  ***** ADC Configuration ***** //
    ADC14_enableModule(); // initialize module
    ADC14_setResolution(ADC_10BIT); // bit res = 10
    ADC14_initModule(ADC_CLOCKSOURCE_SMCLK,ADC_PREDIVIDER_1,ADC_DIVIDER_1, false); // tie to SMCLK with no divider
    ADC14_configureSingleSampleMode(ADC_MEM0, false); // Place Result of Conversion in ADC_MEM0 register
    ADC14_configureConversionMemory(ADC_MEM0, ADC_VREFPOS_INTBUF_VREFNEG_VSS, ADC_INPUT_A0, false);
    REF_A_setReferenceVoltage(REF_A_VREF2_5V);
    ADC14_enableSampleTimer(ADC_MANUAL_ITERATION);
    ADC14_enableConversion(); // ENC Bit

    // ***** Timer A1 Set Up ***** //
    Timer_A_configureUpMode(TIMER_A1_BASE, &upConfig); // Timer_A1 NOT A0 (because thats in use??)
    Interrupt_enableInterrupt(INT_TA1_0);               // Enable A interrupt
    Timer_A_startCounter(TIMER_A1_BASE,TIMER_A_UP_MODE); // Start Timer A

    Interrupt_enableMaster(); // enable all interrupts

    // PWM Configuration
    Timer_A_generatePWM(TIMER_A0_BASE, &pwmConfig);
    UART_sendString("\r\n Select Setpoint from 3'' to 36'' : \n\r");   // Promot User and reset setpoint


    while(1){


        // ***** CONTROLLING MOTOR ON/OFF ***** //
        // States of S1 and S2
        sw1_old = sw1_cur;
        sw1_cur = GPIO_getInputPinValue(GPIO_PORT_P1,GPIO_PIN4);
        //printf("\n sw old: %u --- sw current: %u", sw1_old, sw1_cur);

        // On and Off Tracking
        if( sw1_cur == 0 && sw1_old == 1){
            onoff1++;
            if(onoff1 > 1)
                onoff1 = 0;
        }

        // Turns on Motor and Light when SW1 ON
        if( onoff1 == 1 ){ // if remainder is 1 ... switch 1 is ON
            GPIO_setOutputHighOnPin(LED2,BLUE);
            TIMER_A0->CCR[1] = 2500;    // pin 2.4 gets PWM voltage
        } else{
            GPIO_setOutputLowOnPin(LED2,BLUE);
            TIMER_A0->CCR[1] = 0;
        }

        // ***** READING FROM UART ***** //
        //            setPoint
        if(!readingUART){
            printf("\n setpoint: %u", setPoint);
        }



    } // end while
} // end main


///////////////////// FUNCTIONS / INERRUPTS ///////////////////////


// Timer Interrupt Handler - triggers at every period set in confiH
void TA1_0_IRQHandler(void) // checks setpoint from PUTTY Eventually...
{

    Timer_A_clearCaptureCompareInterrupt(TIMER_A1_BASE, TIMER_A_CAPTURECOMPARE_REGISTER_0);
}

// UART Interrupt Handler - triggers when register is flagged
void EUSCIA0_IRQHandler(){

    uint32_t status = UART_getEnabledInterruptStatus(EUSCI_A0_BASE); // checks that the UART int is ENAB

    if (status &  EUSCI_A_UART_RECEIVE_INTERRUPT_FLAG){ // if ENAB AND Flag
        // UART receive interrupt flag is set
        uint8_t receivedChar = UART_receiveData(EUSCI_A0_BASE);

        if(readingUART){ // if READING

            // Set up for REVEIVED data
            char input[2];
            uint8_t len = 0;

            // Recieved letter process
            while (len < 3  ){
                char c = UART_receivedChar();
                if (c == '\r' || c == '\n' || c == ' '){
                    readingUART = false; // no longer reading

                    break;
                }

                UART_sendChar(c);
                input[len++] = c; // this adds the newest character to the
                // end of the input string
            }

            // Input --> Set Point


            if(atoi(input) < 3 || atoi(input) > 36){
                UART_sendString("\r\n Invalid Setpoint. Select number 3 - 36:\r\n ");

            }else {
                input[len] = '\0'; // clears input
                setPoint = atoi(input); // Convert input string to an integer
                UART_sendString("\r\n Setpoint: \r\n");
                UART_sendInt(setPoint);
                UART_sendString("\r\n Select Setpoint from 3'' to 36'' : \n\r");   // Promot User and reset setpoint

            }


        } else {
            readingUART = true;
        } //  end 'if reading'
        UART_clearInterruptFlag(EUSCI_A0_BASE, EUSCI_A_UART_RECEIVE_INTERRUPT_FLAG);
     } // end if enabled and if flagged

} // end of interrupt

