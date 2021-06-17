/***********************************************************************
* @file      main.cpp
*
*    Illustrating how the ARM Mbed Baremetal profile can be used to 
*    control several LEDs concurrently (even though threading is not
*    supported).
* 
*    Functionally, this application continuously blinks 3 10mm external 
*    LEDs connected to various I/O ports at different frequencies, whilst
*    employing PWM output pins to vary the intensity of yet another 3 10mm
*    LEDs according to:
* 
*    [1] An on-the-fly calculated sawtooth waveform pattern,
* 
*    [2] A precomputed triangular waveform pattern,
* 
*    [3] And a precomputed sinusoidal waveform pattern.
*    
*    Essentially, the values of said waveforms are scaled and used as the
*    duty cycle in driving, dimming, and brightening of the latter 3 10mm
*    LEDs.
*
* @brief   Input: None
*          Output: Various LED patterns of continuously changing intensity.
* 
* @note    Bare metal is a profile of ARM Mbed OS for ultraconstrained 
*          devices; compact and without an RTOS. It builds only the smallest
*          set of APIs that applications require - driver APIs, platform
*          APIs and a subset of the RTOS APIs. This gives one better control
*          of the application's final size.
*
* @warning   The I/O pins of the STM32 NUCLEO-F767ZI are 3.3 V compatible 
*            instead of 5 V for say, the Arduino Uno V3 microcontroller.
*            Hence values for current-limiting resistors placed in series
*            with the external 10mm LEDs must be adjusted accordingly.
* 
*            Furthermore, the STM32 GPIO pins are not numbered 1-64; rather, 
*            they are named after the MCU IO port that they are controlled
*            by. Hence PA_5 is pin 5 on port A. This means that physical
*            pin location may not be related to the pin name. Consult 
*            the "Extension connectors" sub-chapter of the "Hardware 
*            layout and configuration" chapter of the STM32 Nucleo-144 
*            boards UM1974 User manual (en.DM00244518.pdf) to know where
*            each pin is located. Note that all those pins shown can be 
*            used as GPIOs, however most of them also have alternative 
*            functions which are indicated on those diagrams.
*
* @author    Nuertey Odzeyem
* 
* @date      June 17th, 2021
*
* @copyright Copyright (c) 2021 Nuertey Odzeyem. All Rights Reserved.
***********************************************************************/
#include "mbed.h"
#include "mbed_events.h"
#include <stdio.h>
#include "waveforms.h"

// Target = NUCLEO-F767ZI:
#define LED_ON  1
#define LED_OFF 0

// As per my ARM STM32 NUCLEO-F767ZI specs, LEDs 'in situ' on the MCU:        
DigitalOut g_LEDGreen(LED1);
DigitalOut g_LEDBlue(LED2);
DigitalOut g_LEDRed(LED3);

// =======================================================================
// 10mm LED connections to NUCLEO-F767ZI output pins are documented below:
//
// Employing 100Ω resistors for these next three per my actual breadboard
// wiring.
// =======================================================================

// Connector: CN10
// Pin      : 12 
// Pin Name : D2
// STM32 Pin: PF15 ***
// Signal   : I/O
DigitalOut g_External10mmLEDGreen(PF_15); // LED Current = 18mA; Voltage Drop = 2.1V; Calculated Resistance = 66.67Ω

// *** CAUTION!!!
// PA7 is used as D11 and connected to CN7 pin 14 by default. If JP6 is ON,
// it is also connected to both Ethernet PHY as RMII_DV and CN9 pin 15. 
// In this case only one function of the Ethernet or D11 must be used.
//
// So choose some other pin here other than CN9, pin 15 (PA7).

// Connector: CN9 
// Pin      : 30 
// Pin Name : D64
// STM32 Pin: PG1
// Signal   : I/O
DigitalOut g_External10mmLEDYellow(PG_1);// LED Current = 18mA; Voltage Drop = 2.1V; Calculated Resistance = 66.67Ω

// Connector: CN10 
// Pin      : 28 
// Pin Name : D38
// STM32 Pin: PE14
// Signal   : I/O
DigitalOut g_External10mmLEDRed(PE_14);   // LED Current = 18mA; Voltage Drop = 2.0V; Calculated Resistance = 72.22Ω

// ====================================================================
// 10mm LED connections to NUCLEO-F767ZI PWM pins are documented below:
//
// Employing 47Ω resistors for these next three per my actual breadboard
// wiring.
// ====================================================================

// Connector: CN7 
// Pin      : 18 
// Pin Name : D9
// STM32 Pin: PD15
// Signal   : TIMER_B_PWM2
PwmOut g_ExternalPWMLEDGreen(PD_15);

// Connector: CN10 
// Pin      : 29 
// Pin Name : D32
// STM32 Pin: PA0
// Signal   : TIMER_C_PWM1
PwmOut g_ExternalPWMLEDYellow(PA_0);

// Connector: CN10 
// Pin      : 31 
// Pin Name : D33
// STM32 Pin: PB0
// Signal   : TIMER_D_PWM1
PwmOut g_ExternalPWMLEDRed(PB_0);
      
// Let's ride on the bleeding edge with chrono_literals:
using namespace std::chrono_literals;

// =============================
// Begin Actual Implementations:
// =============================
struct ExternalLED_t 
{
    ExternalLED_t(DigitalOut * pExternalLEDPin, const uint32_t& timeOn, const uint32_t& timeOff)
        : m_pExternalLEDPin(pExternalLEDPin) // CAUTION! Ensure that the scope of pExternalLEDPin outlasts us.
        , m_TimeOn(timeOn)
        , m_TimeOff(timeOff)
    {
    }
    ExternalLED_t(const ExternalLED_t& otherLED)
    {
        // CAUTION! Ensure that the scope of the external LED pin pointer
        // outlasts the thread which uses this type; and it DOES, since it
        // is really declared as global. Eschew std::move() then and simply
        // re-point our pointer to that global variable address.
        //m_pExternalLEDPin = std::move(otherLED.m_pExternalLEDPin);
        m_pExternalLEDPin = otherLED.m_pExternalLEDPin;
        m_TimeOn = otherLED.m_TimeOn;
        m_TimeOff = otherLED.m_TimeOff;
    }

    DigitalOut *   m_pExternalLEDPin;
    uint32_t       m_TimeOn;
    uint32_t       m_TimeOff;
};

void LEDBlinker(ExternalLED_t * pExternalLED)
{
    while (1) 
    {
        *(pExternalLED->m_pExternalLEDPin) = LED_ON;
        ThisThread::sleep_for(pExternalLED->m_TimeOn);
        *(pExternalLED->m_pExternalLEDPin) = LED_OFF;
        ThisThread::sleep_for(pExternalLED->m_TimeOff);
    }
}

void LEDSawToothWave(PwmOut * pExternalLEDPin)
{
    while (1)
    {
        // Gradually change the intensity of the LED according to the
        // saw-tooth waveform pattern.
        *pExternalLEDPin = *pExternalLEDPin + 0.01;
        ThisThread::sleep_for(200);

        // Set the output duty-cycle, specified as a percentage (float)
        //
        // Parameters
        //    value A floating-point value representing the output duty-cycle, 
        //    specified as a percentage. The value should lie between 0.0f 
        //    (representing on 0%) and 1.0f (representing on 100%). Values 
        //    outside this range will be saturated to 0.0f or 1.0f.
        if (*pExternalLEDPin == 1.0)
        {
            *pExternalLEDPin = 0;
        }
    }
}

void LEDTriangularWave(PwmOut * pExternalLEDPin)
{
    auto result = std::max_element(g_TriangleWaveform, g_TriangleWaveform + NUMBER_OF_TRIANGULAR_SAMPLES);
    while (1)
    {
        for (auto & dutyCycle : g_TriangleWaveform) 
        {
            // Set the output duty-cycle, specified as a percentage (float)
            //
            // Parameters
            //    value A floating-point value representing the output duty-cycle, 
            //    specified as a percentage. The value should lie between 0.0f 
            //    (representing on 0%) and 1.0f (representing on 100%). Values 
            //    outside this range will be saturated to 0.0f or 1.0f.
            float scaledDutyCycle = (dutyCycle/(*result));
            *pExternalLEDPin = scaledDutyCycle;
            ThisThread::sleep_for(200);
        }
    }
}

void LEDSinusoidalWave(PwmOut * pExternalLEDPin)
{
    auto result = std::max_element(g_SineWaveform, g_SineWaveform + NUMBER_OF_SINUSOID_SAMPLES);
    while (1)
    {
        for (auto & dutyCycle : g_SineWaveform) 
        {
            // Set the output duty-cycle, specified as a percentage (float)
            //
            // Parameters
            //    value A floating-point value representing the output duty-cycle, 
            //    specified as a percentage. The value should lie between 0.0f 
            //    (representing on 0%) and 1.0f (representing on 100%). Values 
            //    outside this range will be saturated to 0.0f or 1.0f.
            float scaledDutyCycle = (dutyCycle/(*result));
            *pExternalLEDPin = scaledDutyCycle;
            ThisThread::sleep_for(40);
        }
    }
}

int main()
{
    printf("\r\n\r\nNuertey-ConcurrentLEDs-Baremetal - Beginning... \r\n\r\n");

    // By default, enough buffer space for 32 Callbacks, i.e. 32*EVENTS_EVENT_SIZE
    // Reduce this amount if the target device has severely limited RAM.
    EventQueue theMasterEventQueue;

    // It seems that Mbed Callback class can only take in one argument.
    // Not to worry, we will improvise with an aggregate class type.
    ExternalLED_t external10mmLEDGreen(&g_External10mmLEDGreen, 100, 100);
    ExternalLED_t external10mmLEDYellow(&g_External10mmLEDYellow, 200, 100);
    ExternalLED_t external10mmLEDRed(&g_External10mmLEDRed, 500, 200);

    // Events are simple callbacks:
    theMasterEventQueue.call(LEDSawToothWave, &g_ExternalPWMLEDGreen);
    theMasterEventQueue.call(LEDTriangularWave, &g_ExternalPWMLEDYellow));
    theMasterEventQueue.call(LEDSinusoidalWave, &g_ExternalPWMLEDRed);
    theMasterEventQueue.call(LEDBlinker, &external10mmLEDYellow);
    theMasterEventQueue.call(LEDBlinker, &external10mmLEDRed);
    theMasterEventQueue.call(LEDBlinker, &external10mmLEDGreen);

    // We will never return from the call below, as events are executed by 
    // the dispatch_forever method. And this is precisely what we want as,    
    // to be compatible with ARM microlib, a bare metal application MUST
    // NEVER return from main(). A crash will occur otherwise!
    theMasterEventQueue.dispatch_forever();

    // So we will NEVER get to this statement. Great! It is per design.
    printf("\r\n\r\nNuertey-ConcurrentLEDs-Baremetal Application - Exiting.\r\n\r\n");
}
