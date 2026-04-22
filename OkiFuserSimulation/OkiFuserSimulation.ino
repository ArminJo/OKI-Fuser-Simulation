/*
 *  OkiFuserSimulation.cpp
 *
 *  Simulates the Fuser in an OKI C833 color laser printer
 *
 *  Copyright (C) 2026  Armin Joachimsmeyer
 *  armin.joachimsmeyer@gmail.com
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 *  See the GNU General Public License for more details.

 *  You should have received a copy of the GNU General Public License
 *  along with this program. If not, see <http://www.gnu.org/licenses/gpl.html>.
 *
 */

/*
 * First, the voltage at the Fuser "Lower" thermistor is measured to determine if Fuser is attached.
 * If Fuser is NOT attached, we measure 0 V.
 *
 * CASE FUSER ATTACHED:
 * The voltages at the Fuser pins and the level at digital pins are measured.
 * The values are printed every 1/10 second for 50 seconds in Arduino1.8.19 Serial Plotter format.
 *
 * CASE FUSER NOT ATTACHED / FUSER SIMULATION:
 * !!! The Opto-interrupter must be closed / interrupted manually by a piece inserted in the slot!!!
 * - Analog inputs are initially driven HIGH to simulate cold Fuser thermistors.
 * - The opto output is driven LOW to simulate an open (unblocked) interrupter.
 * - When backward motor movement is detected, the opto output is released
 *   to simulate a closed (blocked) interrupter.
 * - When the triac (originally used to switch the Fuser heating lamp) is activated:
 *     • Digital lines are driven HIGH to simulate a hot Fuser.
 *     • Analog lines are switched back to input mode.
 * - If no triac activation occurs for 60 seconds, the system resets to the
 *   cold Fuser thermistors simulation:
 *
 *
 *
 *             OKI POWER CONNECTOR PINOUT
 *                    ?  2 -   - 1  5V
 *                   5V  4 -   - 3  5V
 *         Logic Ground  6 -   - 5  Logic Ground
 *         Logic Ground  8 -   - 7  Logic Ground
 *              24 Volt 10 -   - 9  24 Volt
 *              24 Volt 12 -   - 11 24 Volt
 *         Power Ground 14 -   - 13 Power Ground
 *         Power Ground 16 -   - 15 Power Ground
 *      OptoTriac 1200W 18 -   - 17 10 k Thermistor
 * VCC for 2 OptoTriacs 20 -   - 19 OptoTriac 500W
 *        3.4 V Standby 22 -   - 21 Main On Indication
 *    OptoTriac Main On 24 -   - 23 Power Ground
 *
 *    Prototype board connector pinout:
 *    GND | 5V | OPTO Collector | TRIAC1 | TRIAC2 | THERMISTOR1 | TH2 | TH3 | TH4 | MOTOR | BACKWARDS | GND
 *
 *    Thermistor are connected by "cold" resistors to A0 - A3 and by "hot" resistors to D8 - D11 and A4 - A7.
 *    Opto collector signal is connected with 2.2 kOhm to pin in order to enable pull down.
 */

#include <Arduino.h>

#include "ADCUtils.hpp"

#define VERSION_EXAMPLE "1.0"

// Helper macro for getting a macro definition as string
#if !defined(STR_HELPER) && !defined(STR)
#define STR_HELPER(x) #x
#define STR(x) STR_HELPER(x)
#endif

/*
 * Thermistors are connected to 3.35 Volt
 * Channel 0 (heater) has a pull down of 21 kOhm
 * Channels 1 to 3 have a pull down of 5.6 kOhm
 */
//   Measured resistance of Fuser at 20 degree      Resistor values for simulation
//                                            20°C  Cold  Hot  Description               Type        Fuser Connector Relay Connector
#define HEATER_THERMISTOR_CHANNEL       0  // 1M5   470k  33k  Heater Thermistor         PM5-342     A7              18
#define UPPER_CENTER_THERMISTOR_CHANNEL 1  // 310k  100k 6.8k  Upper Center Thermistor   PT5-312     A5              20
#define LOWER_THERMISTOR_CHANNEL        2  // 310k  100k  10k  Lower (case) Thermistor   PTA7-312    B5              24
#define UPPER_SIDE_THERMISTOR_CHANNEL   3  // 310k  100k 5.6k  Upper Side Thermistor     PTA7-312    B3              26
#define CHANNEL_OFFSET_THERMISTOR_COLD  4  // Add this to channels, if thermistors are set to cold, i.e. A0 to A3 are used as digital outputs.
#define HEATER_THERMISTOR_PIN           A0 // Connected with 470k to Relay connector pin 18
#define UPPER_CENTER_THERMISTOR_PIN     A1
#define LOWER_THERMISTOR_PIN            A2
#define UPPER_SIDE_THERMISTOR_PIN       A3
#define LOWER_THERMISTOR_ATTACHED_MILLIVOLT 40 // If Fuser is attached we see around 80 to 140 mV
/*
 * The pins, which are set to VCC / HIGH, to simulate the cold thermistors
 * VCC is OKI 5V and a shottky diode.
 */
#define HEATER_THERMISTOR_HOT_PIN        8 // Connected with 33k to Relay connector pin 18
#define UPPER_CENTER_THERMISTOR_HOT_PIN  9
#define LOWER_THERMISTOR_HOT_PIN        10
#define UPPER_SIDE_THERMISTOR_HOT_PIN   11
#define EXTERNAL_VCC_PIN                12 // For testing purposes, when 5V is provided per USB. Booting waits for it to be high.
bool ThermistorColdIsActivated = false;

/*
 * Opto interrupter has a pull up of 33 kOhm to 3.25 Volt
 */
#define MOTOR_FEEDBACK_PIN              2 // External LED output
#define MOTOR_BACKWARD_PIN              3
#define MOTOR_PIN                       4
#define OPTO_INTERRUPTOR_COLLECTOR_PIN  5 // Pin 15 at Relay connector. Used as input and output. Connected with 2.2 kOhm.
#define TRIAC1_PIN                      6
#define TRIAC2_PIN                      7 // Not really needed

uint16_t sHeaterMillivolt;      // Pin 18 at Relay connector
uint16_t sUpperCenterMillivolt; // Pin 20 at Relay connector
uint16_t sLowerMillivolt;       // Pin 24 at Relay connector. This thermistor has a slow reaction to heating, it maybe measures the case temperature
uint16_t sUpperSideMillivolt;   // Pin 26 at Relay connector
bool sMotorBackwardInput;       // Active high signal
bool sMotorInput;               // Rectangle signal
bool sOptoInput;                // Pin 15 at Relay connector
bool sTriac1Input;              // Influences mainly the UpperSide value. Connected to driving transistor above Power connector
bool sTriac2Input;              // Influences the Heater, Center and UpperSide value. Connected to driving transistor above Power connector

/*
 * Listener mode handling
 */
bool sInSimulationMode = false;                // Simulate the Fuser, since it seems not to be inserted
#define MILLIS_BEFORE_LED_LOW             4000 // 4 second before internal LED which indicates the simulation mode goes low
bool sBootChecksAreDone = false;
#define MILLIS_BETWEEN_MEASUREMENTS        100 // 10 measurements per second
uint32_t sLastMeasurementMillis = 0;

#define DELAY_MOTOR_OFF_MILLIS            2000 // motor signal must be 2 seconds without changes for motor off detection
enum MotorStateEnum {
    MOTOR_OFF = 0, MOTOR_ON
};
MotorStateEnum sMotorState = MOTOR_OFF;
bool sMotorRunsBackward;
uint8_t sMotorSignalChangeCounter = 0;
bool sLastMotorSignalLevel;
uint32_t sMotorTimingMillis;

/*
 * Values for Opto interrupter handling
 * After 11 seconds the interrupter is closed for 400 ms, then opened for 1300 ms and then closed until power down, where it is opened again.
 */
#define DELAY_AFTER_MOTOR_BACKWARD_MILLIS            700
#define DURATION_OF_FIRST_OPTO_OPEN_MILLIS           400
#define DURATION_OF_OPTO_CLOSE_MILLIS               1300 // 1300 is measured value for Fuser inserted detection

enum OptoInterrupterStateEnum {
    INTERRUPTER_WAITING_FOR_MOTOR_BACKWARD = 0,
    INTERRUPTER_WAITING_FOR_CLOSE,
    INTERRUPTER_CLOSED,
    INTERRUPTER_OPENED,
    INTERRUPTER_WAITING_FOR_MOTOR_FORWARD
};
OptoInterrupterStateEnum sOptoInterrupterState = INTERRUPTER_WAITING_FOR_MOTOR_BACKWARD;
uint32_t sOptoInterrupterTimingMillis;

/*
 * Values for thermistor handling
 */
#define DELAY_TO_HOT_AFTER_TRIAC_ON_MILLIS    5000 // 5 seconds
#define DELAY_TO_COLD_AFTER_TRIAC_OFF_MILLIS 60000 // 60 seconds

enum ThermistorStateEnum {
    THERMISTOR_START = 0, THERMISTOR_WAITING_FOR_HOT, THERMISTOR_HOT
};
ThermistorStateEnum sThermistorState = THERMISTOR_START;
uint32_t sThermistorTimingMillis;

void getAnalogValues() {
    readVCCVoltageMillivolt();
    uint8_t tChannelOffset = 0;
    if (ThermistorColdIsActivated) {
        /*
         * Use channels 4 to 7 connected to pins 8 to 11, because here A0 to A3 are set to digital output HIGH level.
         */
        tChannelOffset = CHANNEL_OFFSET_THERMISTOR_COLD;
    }
    sHeaterMillivolt = getVoltageMillivolt(sVCCVoltageMillivolt, HEATER_THERMISTOR_CHANNEL + tChannelOffset);
    sUpperCenterMillivolt = getVoltageMillivolt(sVCCVoltageMillivolt, UPPER_CENTER_THERMISTOR_CHANNEL + tChannelOffset);
    sLowerMillivolt = getVoltageMillivolt(sVCCVoltageMillivolt, LOWER_THERMISTOR_CHANNEL + tChannelOffset);
    sUpperSideMillivolt = getVoltageMillivolt(sVCCVoltageMillivolt, UPPER_SIDE_THERMISTOR_CHANNEL + tChannelOffset);
}
void getDigitalValues() {
    sMotorBackwardInput = digitalRead(MOTOR_BACKWARD_PIN);
    sMotorInput = digitalRead(MOTOR_PIN);
    sOptoInput = digitalRead(OPTO_INTERRUPTOR_COLLECTOR_PIN);
    sTriac1Input = digitalRead(TRIAC1_PIN);
    sTriac2Input = digitalRead(TRIAC2_PIN);
}

void printCaption(uint16_t aCounter) {
    if (sInSimulationMode) {
        Serial.print(F("Simulation"));
    } else {
        Serial.print(F("Listener"));
    }
    Serial.print(F("Counter_"));
    Serial.print(aCounter);
    Serial.print(F("=0 Heater[mV]="));
    Serial.print(sHeaterMillivolt);
    Serial.print(F(" UpperCenter="));
    Serial.print(sUpperCenterMillivolt);
    Serial.print(F(" Lower="));
    Serial.print(sLowerMillivolt);
    Serial.print(F(" UpperSide="));
    Serial.print(sUpperSideMillivolt);
    Serial.print(F(" Motor="));
    Serial.print(sMotorInput);
    Serial.print(F(" Backward="));
    Serial.print(sMotorBackwardInput);
    Serial.print(F(" Opto="));
    Serial.print(sOptoInput);
    Serial.print(F(" Triac1="));
    Serial.print(sTriac1Input);
    Serial.print(F(" Triac2="));
    Serial.print(sTriac2Input);
    Serial.println();
}

void printValues(uint16_t aCounterToPrint) {
    Serial.print(aCounterToPrint * 32);
    Serial.print(' ');
    Serial.print(sHeaterMillivolt);
    Serial.print(' ');
    Serial.print(sUpperCenterMillivolt);
    Serial.print(' ');
    Serial.print(sLowerMillivolt);
    Serial.print(' ');
    Serial.print(sUpperSideMillivolt);
    Serial.print(' ');
    Serial.print(sMotorInput * 400);
    Serial.print(' ');
    Serial.print(sMotorBackwardInput * 500);
    Serial.print(' ');
    Serial.print(sOptoInput * 600);
    Serial.print(' ');
    Serial.print(sTriac1Input * 800);
    Serial.print(' ');
    Serial.print(sTriac2Input * 1000);
    Serial.println();
}

uint16_t sPrintIndex = 0;

/*
 * Get values of all inputs and print them each 1/10 second in Arduino IDE 1.8.19 Serial Plotter format
 */
void doPrintValues() {
    getAnalogValues(); // Values are only required for printing!

    if ((sPrintIndex % 100) == 0) {
        printCaption(sPrintIndex);
    } else {
        printValues(sPrintIndex % 10);
    }
}

/*
 * Print 501 values every 100 ms and then stop
 * Always increment sPrintIndex
 */
void doPrintValuesPeriodically() {
    if (millis() - sLastMeasurementMillis > MILLIS_BETWEEN_MEASUREMENTS) {
        sLastMeasurementMillis = millis();
        if (sPrintIndex <= 500) {
            doPrintValues();
        }
        sPrintIndex++;
        // wait forever
        if (sPrintIndex > 501) {
            sPrintIndex = 501; // 500 triggers printing of caption
        }
    }
}

void activateColdThermistors() {
    /*
     * Set the analog pins to digital output and high to simulate the cold thermistors
     */
    pinMode(HEATER_THERMISTOR_PIN, OUTPUT);
    digitalWrite(HEATER_THERMISTOR_PIN, HIGH);
    pinMode(UPPER_CENTER_THERMISTOR_PIN, OUTPUT);
    digitalWrite(UPPER_CENTER_THERMISTOR_PIN, HIGH);
    pinMode(LOWER_THERMISTOR_PIN, OUTPUT);
    digitalWrite(LOWER_THERMISTOR_PIN, HIGH);
    pinMode(UPPER_SIDE_THERMISTOR_PIN, OUTPUT);
    digitalWrite(UPPER_SIDE_THERMISTOR_PIN, HIGH);
    ThermistorColdIsActivated = true;
}

void deactivateColdThermistors() {
    pinMode(HEATER_THERMISTOR_PIN, INPUT);
    pinMode(UPPER_CENTER_THERMISTOR_PIN, INPUT);
    pinMode(LOWER_THERMISTOR_PIN, INPUT);
    pinMode(UPPER_SIDE_THERMISTOR_PIN, INPUT);
    ThermistorColdIsActivated = false;
}

void activateHotThermistors() {
    /*
     * Activate the outputs to simulate the hot thermistors
     */
    pinMode(HEATER_THERMISTOR_HOT_PIN, OUTPUT);
    digitalWrite(HEATER_THERMISTOR_HOT_PIN, HIGH);
    pinMode(UPPER_CENTER_THERMISTOR_HOT_PIN, OUTPUT);
    digitalWrite(UPPER_CENTER_THERMISTOR_HOT_PIN, HIGH);
    pinMode(LOWER_THERMISTOR_HOT_PIN, OUTPUT);
    digitalWrite(LOWER_THERMISTOR_HOT_PIN, HIGH);
    pinMode(UPPER_SIDE_THERMISTOR_HOT_PIN, OUTPUT);
    digitalWrite(UPPER_SIDE_THERMISTOR_HOT_PIN, HIGH);
//    digitalWrite(LED_BUILTIN, HIGH);
}

void deactivateHotThermistors() {
    /*
     * Activate the outputs to simulate the hot thermistors
     */
    pinMode(HEATER_THERMISTOR_HOT_PIN, INPUT);
    pinMode(UPPER_CENTER_THERMISTOR_HOT_PIN, INPUT);
    pinMode(LOWER_THERMISTOR_HOT_PIN, INPUT);
    pinMode(UPPER_SIDE_THERMISTOR_HOT_PIN, INPUT);
//    digitalWrite(LED_BUILTIN, LOW);
}

/*
 * Count signal changes
 */
void handleMotorState() {
    if (sLastMotorSignalLevel != sMotorInput) {
        sLastMotorSignalLevel = sMotorInput;
        sMotorSignalChangeCounter++;
    }
    bool tMotorSignalIsActive = false;
    if (sMotorSignalChangeCounter > 10) { // 32 is too much
        tMotorSignalIsActive = true;
        sMotorSignalChangeCounter = 0;
    }

    switch (sMotorState) {
    case MOTOR_OFF:
        if (tMotorSignalIsActive) {
            Serial.println(F("Motor_on"));
            sMotorState = MOTOR_ON;
            sMotorTimingMillis = millis();
        }
        break;
    case MOTOR_ON:
        if (tMotorSignalIsActive) {
            sMotorTimingMillis = millis(); // Motor is still on -> rearm delay
        } else {
            if (millis() - sMotorTimingMillis > DELAY_MOTOR_OFF_MILLIS) {
                Serial.println(F("Motor_off_after_delay"));
                sMotorState = MOTOR_OFF;
            }
        }
        break;

    default:
        break;
    }
    /*
     * Motor backward detection
     */
    sMotorRunsBackward = (sMotorState == MOTOR_ON) && sMotorBackwardInput;
    digitalWrite(MOTOR_FEEDBACK_PIN, sMotorRunsBackward);
}

/*
 * Pull the collector low to simulate an open interrupter.
 * This simulates the LED illuminating the transistor, which then becomes conductive.
 */
void openOptoInterrupter() {
    pinMode(OPTO_INTERRUPTOR_COLLECTOR_PIN, OUTPUT);
    digitalWrite(OPTO_INTERRUPTOR_COLLECTOR_PIN, LOW);
    digitalWrite(LED_BUILTIN, HIGH);
}

/*
 * Set the pin as input to enable the opto-interrupter’s unaffected operation
 */
void closeOptoInterrupter() {
    pinMode(OPTO_INTERRUPTOR_COLLECTOR_PIN, INPUT);
    digitalWrite(LED_BUILTIN, LOW);
}
/*
 * The interrupter opens for 1300 ms
 */
void handleOptoInterrupter() {
    switch (sOptoInterrupterState) {
    case INTERRUPTER_WAITING_FOR_MOTOR_BACKWARD:
        if (sMotorRunsBackward) {
            Serial.println(F("Motor_backward"));
            sOptoInterrupterState = INTERRUPTER_WAITING_FOR_CLOSE;
            sOptoInterrupterTimingMillis = millis();
        }
        break;
    case INTERRUPTER_WAITING_FOR_CLOSE:
        if (millis() - sOptoInterrupterTimingMillis > DELAY_AFTER_MOTOR_BACKWARD_MILLIS) {
            closeOptoInterrupter(); // collector goes high, if interrupter is closed physically
            Serial.println(F("Close_Opto"));
            sOptoInterrupterState = INTERRUPTER_CLOSED;
            if (digitalRead(OPTO_INTERRUPTOR_COLLECTOR_PIN) != HIGH) {
                Serial.println(F("ERROR_Opto_interrupter_is_not_closed_physically"));
                /*
                 * ERROR Opto interrupter is NOT closed physically
                 */
                while (digitalRead(OPTO_INTERRUPTOR_COLLECTOR_PIN) != HIGH) {
                    digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
                    delay(500);
                }
            }
            sOptoInterrupterTimingMillis = millis();
        }
        break;
    case INTERRUPTER_CLOSED:
        if (millis() - sOptoInterrupterTimingMillis > DURATION_OF_FIRST_OPTO_OPEN_MILLIS) {
            if (!sTriac1Input) {
                // Triac is active, then we do not see the 1.3 second pulse
                Serial.println(F("Skip_open_Opto"));
                sOptoInterrupterState = INTERRUPTER_WAITING_FOR_MOTOR_FORWARD;
            } else {
                Serial.println(F("Open_Opto"));
                openOptoInterrupter(); // collector forced low for DURATION_OF_OPTO_CLOSE_MILLIS
                sOptoInterrupterState = INTERRUPTER_OPENED;
                sOptoInterrupterTimingMillis = millis();
            }
        }
        break;
    case INTERRUPTER_OPENED:
        if (millis() - sOptoInterrupterTimingMillis > DURATION_OF_OPTO_CLOSE_MILLIS) {
            Serial.println(F("Close_Opto_again"));
            closeOptoInterrupter(); // collector goes high, if interrupter is closed physically
            sOptoInterrupterState = INTERRUPTER_WAITING_FOR_MOTOR_FORWARD;
        }
        break;
    case INTERRUPTER_WAITING_FOR_MOTOR_FORWARD:
        if (!sMotorRunsBackward) {
            Serial.println(F("Motor_forward_do_start_new_sequence"));
            sOptoInterrupterState = INTERRUPTER_WAITING_FOR_MOTOR_BACKWARD;
        }
        break;
    default:
        break;
    }
}

void handleThermistorSimulation() {
    bool tOneTriacIsActive = !(sTriac1Input && sTriac2Input); // Triac values are active low
//    digitalWrite(TRIAC_FEEDBACK_PIN, tOneTriacIsActive);

    switch (sThermistorState) {
    case THERMISTOR_START:
        if (tOneTriacIsActive) {
            // At least one TRIAC was activated, delay before HOT
            Serial.println(F("Triac_just_activated"));
            sThermistorState = THERMISTOR_WAITING_FOR_HOT;
            sThermistorTimingMillis = millis();
        }
        break;
    case THERMISTOR_WAITING_FOR_HOT:
        if (!tOneTriacIsActive) {
            // reset state before activating thermistors
            Serial.println(F("Triac_just_deactivated"));
            sThermistorState = THERMISTOR_START;
        }
        if (millis() - sThermistorTimingMillis > DELAY_TO_HOT_AFTER_TRIAC_ON_MILLIS) {
            // Now switch to HOT
            Serial.println(F("Set_thermistors_to_hot"));
            activateHotThermistors();
            deactivateColdThermistors();
            sThermistorState = THERMISTOR_HOT;
            sThermistorTimingMillis = millis();
        }
        break;
    case THERMISTOR_HOT:
        if (tOneTriacIsActive) {
            sThermistorTimingMillis = millis(); // rearm delay for triac going cold
        } else if (millis() - sThermistorTimingMillis > DELAY_TO_COLD_AFTER_TRIAC_OFF_MILLIS) {
            // Simulate cold thermistors. Not sure if really needed.
            Serial.println(F("Set_thermistors_to_cold"));
            activateColdThermistors();
            deactivateHotThermistors();
            sThermistorState = THERMISTOR_START;
            sThermistorTimingMillis = millis();
        }
        break;
    default:
        break;
    }
}

void setup() {
    // initialize the digital pin as an output.
    pinMode(LED_BUILTIN, OUTPUT);
    digitalWrite(LED_BUILTIN, LOW);

//    pinMode(TRIAC_FEEDBACK_PIN, OUTPUT);
//    digitalWrite(TRIAC_FEEDBACK_PIN, LOW);
    pinMode(MOTOR_FEEDBACK_PIN, OUTPUT);
    digitalWrite(MOTOR_FEEDBACK_PIN, LOW);

    Serial.begin(115200);

    /*
     * Only for development - wait for OKI VCC to be high
     */
    if (!digitalRead(EXTERNAL_VCC_PIN)) {
        Serial.println(F("Wait_for_Oki_5V_VCC_going_high"));
        while (!digitalRead(EXTERNAL_VCC_PIN)) {
            digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
            delay(200);
        }
        Serial.println(F("Oki_5V_VCC_is_high"));
    }

    delay(200); // For external analogs to settle
    getDigitalValues();
    getAnalogValues();
    printCaption(0);

    /*
     * Fuser detection
     */
    if (sLowerMillivolt < LOWER_THERMISTOR_ATTACHED_MILLIVOLT) {
        // Fuser is not inserted enter simulation mode
        sInSimulationMode = true;
        // Set Opto to open, this is state if fuser was inserted at last power down
        openOptoInterrupter();
        activateColdThermistors();
        // Just to know which program is running on my Arduino
        Serial.println(F("START " __FILE__ "\r\nVersion " VERSION_EXAMPLE " from " __DATE__));
        Serial.print(F("Start_simulating_Fuser_Lower_Thermistor="));
        Serial.print(sLowerMillivolt);
        Serial.println(F("_mV"));
        Serial.println(F("Set_opto_interrupter_output_after_startup"));
        Serial.println(F("Set_4_thermistor_values_according_to_heating_Triac_signals"));
        digitalWrite(LED_BUILTIN, HIGH);
    } else {
        /*
         * Stay in listener mode, let the interrupter unaffected
         */
        closeOptoInterrupter();
        digitalWrite(LED_BUILTIN, LOW);
    }

#if defined(__AVR_ATmega32U4__) || defined(SERIAL_PORT_USBVIRTUAL) || defined(SERIAL_USB) /*stm32duino*/|| defined(USBCON) /*STM32_stm32*/ \
        || defined(SERIALUSB_PID)  || defined(ARDUINO_ARCH_RP2040) || defined(ARDUINO_attiny3217)
    delay(4000); // To be able to connect Serial monitor after reset or power up and before first print out. Do not wait for an attached Serial Monitor!
#endif
}

void loop() {
    getDigitalValues();
    doPrintValuesPeriodically();
    handleMotorState(); // to see the backward LED :-)

    if (sInSimulationMode) {
        if (!sBootChecksAreDone && millis() > MILLIS_BEFORE_LED_LOW) {
            sBootChecksAreDone = true;
            digitalWrite(LED_BUILTIN, LOW); // do it once after startup
        }
        handleOptoInterrupter();
        handleThermistorSimulation();
    }
}
