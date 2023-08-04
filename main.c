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
#include <string.h>
#include "msp.h"
#include "driverlib.h"
#include "myHeaderFile.h"
#include <math.h>

///////////////////// SETUP ///////////////////////
// DEFINE
#define LED1 GPIO_PORT_P1
#define LED2 GPIO_PORT_P2
#define RED  GPIO_PIN0
#define GREEN  GPIO_PIN1
#define BLUE  GPIO_PIN2
#define SCALE 1000

// GLOBAL VARIABLES
const uint16_t ta0ccr0 = 3000;
volatile float dutyCycle = 0; // subject to change based on interrupt
volatile float pot_v;
volatile float ADCresult;
volatile float height;

unsigned short sw1_cur = 1; // current state of click
unsigned short sw1_old = 1; // previous state of click
unsigned short onoff1 = 0; // checks if SW1 'on' or 'off'

float highest = 42; // highest setpoint
float lowest = 0; // lowest setpoint
volatile float prevHeight = 0; // for hysteresis
volatile float hyst = 5; // Hysteresis

volatile uint16_t setPoint = 0; // Set point
volatile bool readingUART = false; // monitors if UART is reading or writing

float Kp = 10.0; // Proportional gain
float Ki = 0.0; // Integral gain
float Kd = 5.0; // Derivative gain

volatile float error = 0; // error
float prev_error = 0; // previous error
float integralTerm = 0; // tracks integral
float der = 0; // tracks derivative error
volatile float controlSignal = 0.0; // control signal

volatile uint32_t dataSendTime = 0; // how often hegiht gets sent
volatile uint32_t currentTimeMillis = 0; // milliseconds

int i = 0;

// TIMER
const Timer_A_UpModeConfig upConfig =
{
 TIMER_A_CLOCKSOURCE_SMCLK,          // Frequency = 3 MHz
 TIMER_A_CLOCKSOURCE_DIVIDER_16,     // 3 MHz / 16 = 187500 Hz (CCR0 counts up this many times per second)
 18750,                              // This is CCR0: used to set the timer period: 18750 / 187500 = 0.1s period = 10 Hz
 TIMER_A_TAIE_INTERRUPT_DISABLE,
 TIMER_A_CCIE_CCR0_INTERRUPT_ENABLE,
 TIMER_A_DO_CLEAR
};

// PWM
Timer_A_PWMConfig pwmConfig =
{
 TIMER_A_CLOCKSOURCE_SMCLK,          // Frequency = 3 MHz
 TIMER_A_CLOCKSOURCE_DIVIDER_1,      // 3 MHz / 1 = 3 MHz
 ta0ccr0,                               // TA2CCR0: 3000/ (1) / (3,000,000 Hz) = (0.001 s ) PWM period = 1000
 TIMER_A_CAPTURECOMPARE_REGISTER_3 , // TA2CCR1 Resister - pin 2.4 T0.1
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
    initialize_LEDS(LED1,RED);
    initialize_LEDS(LED2,RED|GREEN|BLUE);

    //for timers
    WDT_A_holdTimer(); // stop  s WDT
    FPU_enableModule();
    CS_setDCOFrequency(3E+6); // 3 MHz
    CS_initClockSignal(CS_SMCLK,CS_DCOCLK_SELECT,CS_CLOCK_DIVIDER_1);


    // Disable all Interrupts
       Interrupt_disableMaster();

    // ***** Set Inputs & Outputs ***** //

    GPIO_setAsInputPinWithPullUpResistor(GPIO_PORT_P1,GPIO_PIN1); // S1 as an input
    GPIO_setAsInputPinWithPullUpResistor(GPIO_PORT_P1,GPIO_PIN4); // S2 as an input

    GPIO_setAsPeripheralModuleFunctionInputPin(GPIO_PORT_P5,GPIO_PIN5,GPIO_TERTIARY_MODULE_FUNCTION); // ADC Pin P5.5 - for sensor feedback
    GPIO_setAsPeripheralModuleFunctionOutputPin(GPIO_PORT_P2,GPIO_PIN6,GPIO_PRIMARY_MODULE_FUNCTION); // P2.4 --> Out to fan

    // ***** UART COM5 SET UP ***** // (from Lab 4- Putty and UART)
    GPIO_setAsPeripheralModuleFunctionInputPin(GPIO_PORT_P1, GPIO_PIN2 | GPIO_PIN3, GPIO_PRIMARY_MODULE_FUNCTION);// GPIO and UART TX/RX
    UART_initModule(EUSCI_A0_BASE, &UART_init); //( module , configuration)
    UART_enableModule(EUSCI_A0_BASE);// Initialize UART 0
    UART_enableInterrupt(EUSCI_A0_BASE, EUSCI_A_UART_RECEIVE_INTERRUPT);    // Enable UART receive interrupts
    Interrupt_enableInterrupt(INT_EUSCIA0);


//    // ***** UART COM6 SET UP *****//
//    UART_initModule(EUSCI_A1_BASE, &UART_init);
//    UART_enableModule(EUSCI_A1_BASE);
//
//    const char* comPortName = "COM6"; // set COM port
//
//    // Check if the serial port was successfully opened
//    if (EUSCI_A1->CTLW0 & EUSCI_A_CTLW0_SWRST){
//        // Handle error (unable to open the COM port)
//        printf("Error: Unable to open COM port %s\n", comPortName);
//        return;
//    }


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

    // Set up interrupt priorities
    NVIC_SetPriority(TA1_0_IRQn, 1);          // Timer A1 interrupt priority set to 1
    NVIC_SetPriority(EUSCIA0_IRQn, 0);       // EUSCI A0 UART interrupt priority set to 2

    // Enable interrupts
    Interrupt_enableInterrupt(TA1_0_IRQn);    // Enable Timer A1 interrupt
    Interrupt_enableInterrupt(EUSCIA0_IRQn);  // Enable EUSCI A0 UART interrupt

    // Enable Global Interrupts
    Interrupt_enableMaster(); // enable all interrupts

    // PWM Configuration
    Timer_A_generatePWM(TIMER_A0_BASE, &pwmConfig);

    // Set initial state
    setPoint = 0;
    UART_sendString("\r\n Select Setpoint from 0'' to 42'' : \n\r");   // Promot User and reset setpoint
    dataSendTime= currentTimeMillis;


    while(1){
        // ***** CONTROLLING MOTOR ON/OFF ***** //
        // States of S1 and S2
        sw1_old = sw1_cur;
        sw1_cur = GPIO_getInputPinValue(GPIO_PORT_P1,GPIO_PIN4);

        // On and Off Tracking
        if( sw1_cur == 0 && sw1_old == 1){
            onoff1++;
            if(onoff1 > 1)
                onoff1 = 0;
        }

        // ***** SENDING HEIGHT DATA ***** //
        // If enough time has elapsed, send the height data
        if (currentTimeMillis - dataSendTime >= 5)
        {   dataSendTime = currentTimeMillis;
            char heightStr[16]; // Buffer to store the height data as a string
            snprintf(heightStr, sizeof(heightStr), "%.2f\r\n", height);
//            printf("\r\nHeight String: %s", heightStr);
//            UART_sendString(heightStr);
//            UART_sendString("\r\n TEST ");
            GPIO_toggleOutputOnPin(LED1,RED);

        }

        // ***** SETPOINT PROCESSING ***** //
        // Not used with PID
        dutyCycle = inch2percent( (float)setPoint);


        // ***** FAN OF CONTROL ***** //
        if( onoff1 == 1 ){ // if remainder is 1 ... switch 1 is ON
            GPIO_setOutputHighOnPin(LED2,BLUE);
            printf("\r\n Height: %f ---- Control Signal: %f ---- To Board: %u ---- Error: %f ", height,controlSignal, 2150 + (uint16_t)(controlSignal / 100.0 * 850), error);
//            printf("\r\n Height: %f ---- ADC: %f ", height, ADCresult);
            TIMER_A0->CCR[3] = 2150 + (uint16_t)(controlSignal / 100.0 * 850);
        } else{
            GPIO_setOutputLowOnPin(LED2,BLUE);
            TIMER_A0->CCR[3] = 0;
        }

    } // end while
} // end main


///////////////////// FUNCTIONS / INERRUPTS ///////////////////////

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


            if(atoi(input) < lowest || atoi(input) > highest){
                UART_sendString("\r\n Invalid Setpoint. Select number 0 - 42:\r\n ");

            }else {
                input[len] = '\0'; // clears input
                setPoint = atoi(input); // Convert input string to an integer
                UART_sendString("\r\n Setpoint: ");
                UART_sendInt(setPoint);
                UART_sendString("\r\n Select Setpoint from 0'' to 42'' : \n\r");   // Promot User and reset setpoint

            }
        } else {
            readingUART = true;
        } //  end 'if reading'

        UART_clearInterruptFlag(EUSCI_A0_BASE, EUSCI_A_UART_RECEIVE_INTERRUPT_FLAG);

    } // end if enabled and if flagged
} // end of UART interrupt

// Timer Interrupt Handler - triggers at every period (set in config)
void TA1_0_IRQHandler(){

    // read feed back from ultrasonic sensor
    Timer_A_clearCaptureCompareInterrupt(TIMER_A1_BASE,TIMER_A_CAPTURECOMPARE_REGISTER_0); // clears timer ate A0 interupt
    ADC14_toggleConversionTrigger(); // sets SC / trigger bit

    currentTimeMillis++; // for sending data

    // ***** READING SENSOR ***** //
    while(ADC14_isBusy()){}     // waits for conversion to finish
    ADCresult = ADC14_getResult(ADC_MEM0) ; // read from memory
    ADCresult = (2.5*(float)ADCresult) /1024.0;
    height = (-125 * ADCresult + 52);
    height = (int)round(height);
    if (ADCresult > 0.45) {
        height = 0;
//         printf("Forced: Height = %d\r\n", (int)round(height));
    }

    // ***** CONTROLLER ***** //
    error = (float)setPoint - height ; // Calculate Error
    integralTerm += error;

    // PID control equation
    controlSignal = Kp*error  + Ki*integralTerm + Kd*(error - prev_error); // Update PID

    // Update error for der
    prev_error = error;

    // Saturation Limit
    if (controlSignal < 0 ){
        controlSignal = 0;
    } else if (controlSignal > 100){
        controlSignal = 100;
    }

    // Clear interrupt flag
    Timer_A_clearCaptureCompareInterrupt(TIMER_A1_BASE, TIMER_A_CAPTURECOMPARE_REGISTER_0);

} // end timer interrupt
