#include <stdio.h>
#include <stdlib.h>
#include <p30fxxxx.h>
#include <outcompare.h>
#include <string.h>
#include "adc.h"
#include "tajmer.h"
#include "tajmer_pwm.h"
#include "UART.h"
#include "fotootpornik.h"
#include <stdint.h>
#define BUFFER_SIZE 6

//#define TRIG_NAPRED LATDbits.LATD2
//#define ECHO_NAPRED PORTFbits.RF6

//#define TRIG_NAZAD LATDbits.LATD3
//#define ECHO_NAZAD PORTBbits.RB2


#define TRIG_NAPRED LATDbits.LATD3
#define ECHO_NAPRED PORTBbits.RB2

#define TRIG_NAZAD LATDbits.LATD2
#define ECHO_NAZAD PORTFbits.RF6


#define SPEED_OF_SOUND (0.0343)        // centimetri po mikcrosec.
#define INSTRUCTION_CLOCK_PERIOD (0.1) // microsekundi

_FOSC(CSW_FSCM_OFF &XT_PLL4); // instruction takt je isti kao i kristal 10MHz
_FWDT(WDT_OFF);

unsigned char tempRX1, tempRX2_bluetooth;
unsigned int us_counter;
unsigned int sirovi0, sirovi1, sirovi2, sirovi3;
unsigned int FR_front, FR_back, FR_left, FR_right;
int startFlag = 0;
int stopFlag = 0;

char buffer[BUFFER_SIZE];
unsigned int bufferIndex = 0;

static unsigned char time_overflow_back = 0;
static unsigned char time_overflow_forward = 0; 
static float measured_distance_back = 0;
static float measured_distance_forward = 0;

void __attribute__((__interrupt__, no_auto_psv)) _T1Interrupt(void) // svakih 1us
{
    TMR1 = 0;

    us_counter++; // brojac za funkciju delay_1us()

    IFS0bits.T1IF = 0;
}

void __attribute__((__interrupt__, no_auto_psv)) _T2Interrupt(void) // za PMW
{
    TMR2 = 0;
    IFS0bits.T2IF = 0;
}

void __attribute__((__interrupt__, no_auto_psv)) _T4Interrupt(void) // za dig senzor nazad
{
    TMR4 = 0;
    ECHO_NAZAD = 0;
    time_overflow_back = 1;
    IFS1bits.T4IF = 0;
}

void __attribute__((__interrupt__, no_auto_psv)) _T5Interrupt(void) // za dig senzor napred
{
    TMR5 = 0;
    ECHO_NAPRED = 0;
    time_overflow_forward = 1;
    IFS1bits.T5IF = 0;
}

void __attribute__((__interrupt__, no_auto_psv)) _U2RXInterrupt(void) {
    IFS1bits.U2RXIF = 0;

    tempRX2_bluetooth = U2RXREG;

    //proveri da li je primljeni karakter prelaz u novi red
    //Provera da li je primljen karakter novi red 
    if (tempRX2_bluetooth == '\n' || tempRX2_bluetooth == '\r') {
        // Null- prekida ga
        buffer[bufferIndex] = '\0';

        // Slanje poruke "START"
        if (strcmp(buffer, "START") == 0) {
            //startFlag postavljamo na 1
            startFlag = 1;

        }// Slanje poruke "STOP"
        else if (strcmp(buffer, "STOP") == 0) {
            //stopFlag postavljamo na 1
            stopFlag = 1;
        }
        //Resetujemo bufferIndex za sledeci prijem
        bufferIndex = 0;
    }//Proveravamo da li je buffer pun
    else if (bufferIndex < BUFFER_SIZE - 1) {
        //Dodajemo primljen karakter u bafer
        buffer[bufferIndex] = tempRX2_bluetooth;
        bufferIndex++;
    }
}

void __attribute__((__interrupt__, no_auto_psv)) _U1RXInterrupt(void) {
    IFS0bits.U1RXIF = 0;
    tempRX1 = U1RXREG;
}

void __attribute__((__interrupt__, no_auto_psv)) _ADCInterrupt(void) {

    sirovi0 = ADCBUF0;
    sirovi1 = ADCBUF1;
    sirovi2 = ADCBUF2;
    sirovi3 = ADCBUF3;

    IFS0bits.ADIF = 0;
}

void initPins() {

    // FOTOOTPORNICI analogni
    ADPCFGbits.PCFG0 = 0; // levi-fotoR na RB0
    ADPCFGbits.PCFG1 = 0; // gornji-fotoR na RB1
    ADPCFGbits.PCFG5 = 0; // donji-fotoR na RB5
    ADPCFGbits.PCFG3 = 0; // desni-fotoR na RB3

    TRISBbits.TRISB0 = 1; // ulazni
    TRISBbits.TRISB1 = 1; // ulazni
    TRISBbits.TRISB5 = 1; // ulazni
    TRISBbits.TRISB3 = 1; // ulazni

    // ULTRAZVUCNI NAPRED
    TRISDbits.TRISD2 = 0; // trigger
    TRISFbits.TRISF6 = 1; // echo

    // ULTRAZVUCNI NAZAD
    TRISDbits.TRISD3 = 0; // trigger

    ADPCFGbits.PCFG2 = 1; // digitalni pin
    TRISBbits.TRISB2 = 1; // echo

    // pinovi za kretanje
    TRISFbits.TRISF2 = 0; // in1
    TRISFbits.TRISF3 = 0; // in2
    
    ADPCFGbits.PCFG10 = 1;
    TRISBbits.TRISB10 = 0; // in4
    
    ADPCFGbits.PCFG11 = 1;
    TRISBbits.TRISB11 = 0; // in3

    // pwm
    TRISDbits.TRISD1 = 0; // enB
    TRISDbits.TRISD0 = 0; // enA
}

void delay_1us(int vreme) {
    us_counter = 0;
    T1CONbits.TON = 1;
    while (us_counter < vreme)
        ;
    T1CONbits.TON = 0;
}

void delay_for(uint32_t num) // unsigned int ide do 65535
{
    uint32_t broj;
    for (broj = 0; broj < num; broj++)
        ;
}

static void MeasureBackDistance() {
    //logicka jedinica traje 10us
    TRIG_NAZAD = 1;
    delay_1us(3); //3 od 10 da bi nam logicka jedinica trajala 10us
    TRIG_NAZAD = 0;
    delay_1us(3);
    while (!ECHO_NAZAD)
        ; //echo pin postaje 1(rastuca ivica detektovana)
    TMR4 = 0; // resetuje se tajmer T4
    IFS1bits.T4IF = 0;
    T4CONbits.TON = 1; // ukljucujemo tajmer T4
    while (ECHO_NAZAD)
        ; //echo pin postaje 0(opadajuca ivica detektovana)
    T4CONbits.TON = 0; //isljucujemo tajmer T4
    unsigned int measured_time_back;
    if (time_overflow_back == 1) // 
    {
        measured_time_back = TMR4_period;
        time_overflow_back = 0;
    } else //poslati signal se vratio
    {
        measured_time_back = TMR4;
    }
    TMR4 = 0;
    // delimo sa 2 jer svetlosni signal putuje do prepreke i nazad
    // mnozimo sa INSTRUCTION_CLOCK_PERIOD da bi dobili vreme u mikrosekundama
    measured_distance_back = (measured_time_back * INSTRUCTION_CLOCK_PERIOD) / 2 * SPEED_OF_SOUND;
}

static void MeasureForwardDistance() {
   
    TRIG_NAPRED = 1;
    delay_1us(3); 
    TRIG_NAPRED = 0;
    delay_1us(3);
    while (!ECHO_NAPRED)
        ; 
    TMR5 = 0; 
    IFS1bits.T5IF = 0;
    T5CONbits.TON = 1; 
    while (ECHO_NAPRED)
        ; 
    T5CONbits.TON = 0; 
    unsigned int measured_time_forward;
    if (time_overflow_forward == 1) 
    {
        measured_time_forward = TMR5_period;
        time_overflow_forward = 0;
    } else 
    {
        measured_time_forward = TMR5;
    }
    TMR5 = 0;
    measured_distance_forward = (measured_time_forward * INSTRUCTION_CLOCK_PERIOD) / 2 * SPEED_OF_SOUND;
}

void idiNapred() {

    //   OC1RS=250;
    //  OC2RS=250;
    
    LATFbits.LATF2 = 0; // in1
    LATFbits.LATF3 = 1; // in2
    LATBbits.LATB11 = 1; // in3
    LATBbits.LATB10 = 0; // in4
}

void idiNazad() {

    // OC1RS=385;
    // OC2RS=400;
   
    LATFbits.LATF2 = 1; // in1
    LATFbits.LATF3 = 0; // in2
    LATBbits.LATB11 = 0; // in3
    LATBbits.LATB10 = 1; // in4
}

void idiLevo() {

    //    OC1RS=250;
    //   OC2RS=250;
 
    LATFbits.LATF2 = 1; // in1
    LATFbits.LATF3 = 0; // in2
    LATBbits.LATB11 = 1; // in3
    LATBbits.LATB10 = 0; // in4
}

void idiDesno() {

    //  OC1RS=250;
    // OC2RS=250;
    
    LATFbits.LATF2 = 0; // in1
    LATFbits.LATF3 = 1; // in2
    LATBbits.LATB11 = 0; // in3
    LATBbits.LATB10 = 1; // in4
}

void zaustaviSve() {

    //  OC1RS = 0;
    // OC2RS = 0;
    
    LATFbits.LATF2 = 0; // in1
    LATFbits.LATF3 = 0; // in2
    LATBbits.LATB11 = 0; // in3
    LATBbits.LATB10 = 0; // in4
}
int start_poruka = 0;
int stop_poruka = 0;

int main(int argc, char **argv) {
    initPins();
    ADCinit();
    initUART2(); // bluetooth
    initUART1(); // za serijsku i debugging
    Init_T1();
    Init_T4();
    Init_T5();
    initPWM();

    RS232_putst2("Write START");
    WriteUART2(13);

    while (1) {

        FR_back = fotootpornik(sirovi0); // promenjive za svaki fotootpornik pokazuju da li je on osvjetljen
        FR_right = fotootpornik(sirovi1);
        FR_front = fotootpornik(sirovi2);
        FR_left = fotootpornik(sirovi3);

        if (startFlag == 1 && stopFlag == 0) {

            if (start_poruka == 0) {
                RS232_putst2("Tenk upaljen");
                WriteUART2(13);
                start_poruka = 1;
                stop_poruka = 0;
            }

            if (FR_back == 1 && stopFlag == 0) {

                MeasureBackDistance(); // Izmeri udaljenost unazad
                if (measured_distance_back > 13) {
                    idiNazad(); // Ako je udaljenost unazad veca od 13, kreni nazad
                    while (measured_distance_back >= 13 && stopFlag == 0) {
                        MeasureBackDistance(); // Ponovo izmeri udaljenost unazad
                        if (measured_distance_back < 13) {
                            zaustaviSve(); // Zaustavi tenk ako je udaljenost unazad manja od 13
                        }
                    }
                } else {
                    zaustaviSve(); // Zaustavite tenk ako je udaljenost unazad vec manja od 13
                }
            } else if (FR_right == 1 && stopFlag == 0) {

                idiDesno();
                delay_for(705000); // delay realizovan ovako da se tenk okrene za 90 stepeni
                MeasureForwardDistance(); // Izmeri udaljenost unapred
                if (measured_distance_forward >= 13) {
                    idiNapred(); // Ako je udaljenost unapred veca od 13, kreni napred
                    while (measured_distance_forward >= 13 && stopFlag == 0) {
                        MeasureForwardDistance(); // Ponovo izmeri udaljenost unapred
                        if (measured_distance_forward < 13) {
                            zaustaviSve(); // Zaustavi tenk ako je udaljenost unapred manja od 13
                        }
                    }
                } else {
                    zaustaviSve(); // Zaustavi tenk ako je udaljenost unapred vec manja od 13
                }
            } else if (FR_front == 1 && stopFlag == 0) {

                MeasureForwardDistance(); // Izmeri udaljenost unapred
                if (measured_distance_forward > 13) {
                    idiNapred(); // Ako je udaljenost unapred veca od 13, kreni napred
                    while (measured_distance_forward >= 13 && stopFlag == 0) {
                        MeasureForwardDistance(); // Ponovo izmeri udaljenost unapred
                        if (measured_distance_forward < 13) {
                            zaustaviSve(); // Zaustavi tenk ako je udaljenost unapred manja od 13
                        }
                    }
                } else {
                    zaustaviSve(); // Zaustavi tenk ako je udaljenost unapred vec manja od 13
                }
            } else if (FR_left == 1 && stopFlag == 0) {

                idiLevo();
                delay_for(705000); // delay realizovan ovako da se tenk okrene za 90 stepeni
                MeasureForwardDistance();
                if (measured_distance_forward >= 13) {
                    idiNapred(); // Ako je udaljenost unapred veca od 13, kreni napred
                    while (measured_distance_forward >= 13 && stopFlag == 0) {
                        MeasureForwardDistance(); // Ponovo izmeri udaljenost unapred
                        if (measured_distance_forward < 13) {
                            zaustaviSve(); // Zaustavi tenk ako je udaljenost unapred manja od 13
                        }
                    }
                } else {
                    zaustaviSve(); // Zaustavi tenk ako je udaljenost unapred vec manja od 13
                }
            } else {
                zaustaviSve(); // Zaustavi tenk ako nijedan fotootpornik nije osvetljen
            }
        } else if (stopFlag == 1) {

            if (stop_poruka == 0) {
                RS232_putst2("Tenk ugasen");
                WriteUART2(13);
                stop_poruka = 1;
                start_poruka = 0;
            }
            zaustaviSve();
            stopFlag = 0;
            startFlag = 0;
        }

    } // od while

    return 0;
}
