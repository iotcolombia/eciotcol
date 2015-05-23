/**
 * @file eceiotcol.ino
 * @brief Eggs Container Embedded IoT Colombia
 * @date 23/05/2015
 * @author Leandro Perez Guatibonza / leandropg AT gmail DOT com
 * @author Giovanni García Rodríguez / giovannigarcia DOT de AT gmail DOT com
 * @author Gabriel García / gaengaros18 AT hotmail DOT com
 * @author Jorge Ernesto Guevara Cuneca / guevara DOT ernesto AT gmail DOT com
 * @version 15.4
 *
 * EggBox Embedded IoT Colombia
 * Copyright (C) 2015 - 2016
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 */

/*
 * Multiplexing 12 LEDS using Charlieplexing metod:
 * https://en.wikipedia.org/wiki/Charlieplexing
 *
 */

/**
 * LED Indicator
 */
#define INDICATOR_LED        13

/**
 * Sensors inputs pin (Transistors Colectors)
 */
#define SENSORS_INPUT        11

/**
 * Sensors Power Pin A/B/C/D (Charlieplexing - Diodes emitters)
 */
#define SENSORS_POWER_A      10
#define SENSORS_POWER_B      9
#define SENSORS_POWER_C      8
#define SENSORS_POWER_D      7

/**
 * Multiplexing number of sensors
 */
#define SENSOR_QTY           12

/**
 * Multiplexing number of pins
 */
#define CHARLIEPLEXING       4

/**
 * Pin configuration selector sensors array
 */
#define PIN_CONFIG           0

/**
 * Pin state selector sensors array
 */
#define PIN_STATE            1

/**
 * A/B/C/D selector sensors array
 */
#define A                    0
#define B                    1
#define C                    2
#define D                    3

/**
 * Sensor State Previous
 */
int qtyEggsPrevious = 0;

/**
 * Sensor State Actual
 */
int qtyEggsActual;

/**
 * Sensors settings array:
 *  Set 2 pins to OUTPUT and set only one of them to HIGH to power on a sensor at time 
 */

int sensors[ SENSOR_QTY ][ 2 ][ CHARLIEPLEXING ] = {

  //    PIN_CONFIG                           PIN_STATE
  //    A       B       C      D          A    B    C    D
  { { INPUT, OUTPUT, INPUT, OUTPUT }, { LOW, LOW, LOW, HIGH } },  // SENSOR 1 ( SENSORS_POWER_D SENSORS_POWER_B )
  { { INPUT, OUTPUT, INPUT, OUTPUT }, { LOW, HIGH, LOW, LOW } },  // SENSOR 2 ( SENSORS_POWER_B SENSORS_POWER_D )
  { { INPUT, INPUT, OUTPUT, OUTPUT }, { LOW, LOW, LOW, HIGH } },  // SENSOR 3 ( SENSORS_POWER_D SENSORS_POWER_C )
  { { INPUT, INPUT, OUTPUT, OUTPUT }, { LOW, LOW, HIGH, LOW } },  // SENSOR 4 ( SENSORS_POWER_C SENSORS_POWER_D )
  { { INPUT, OUTPUT, OUTPUT, INPUT }, { LOW, LOW, HIGH, LOW } },  // SENSOR 5 ( SENSORS_POWER_C SENSORS_POWER_B )
  { { INPUT, OUTPUT, OUTPUT, INPUT }, { LOW, HIGH, LOW, LOW } },  // SENSOR 6 ( SENSORS_POWER_B SENSORS_POWER_C )  
  { { OUTPUT, INPUT, INPUT, OUTPUT }, { LOW, LOW, LOW, HIGH } },  // SENSOR 7 ( SENSORS_POWER_D SENSORS_POWER_A )
  { { OUTPUT, INPUT, INPUT, OUTPUT }, { HIGH, LOW, LOW, LOW } },  // SENSOR 8 ( SENSORS_POWER_A SENSORS_POWER_D )
  { { OUTPUT, INPUT, OUTPUT, INPUT }, { LOW, LOW, HIGH, LOW } },  // SENSOR 9 ( SENSORS_POWER_C SENSORS_POWER_A )
  { { OUTPUT, INPUT, OUTPUT, INPUT }, { HIGH, LOW, LOW, LOW } },  // SENSOR 10 ( SENSORS_POWER_A SENSORS_POWER_C )
  { { OUTPUT, OUTPUT, INPUT, INPUT }, { LOW, HIGH, LOW, LOW } },  // SENSOR 11 ( SENSORS_POWER_B SENSORS_POWER_A )
  { { OUTPUT, OUTPUT, INPUT, INPUT }, { HIGH, LOW, LOW, LOW } },  // SENSOR 12 ( SENSORS_POWER_A SENSORS_POWER_B )
};

/**
 * Init Routine
 */
void setup() {
 
  // Configure output egg indicator LED pin
  pinMode(INDICATOR_LED, OUTPUT);
 
  // Configure inputs sensors pin
  pinMode(SENSORS_INPUT, INPUT_PULLUP);

  // Configure Serial Port at 9600 bps
  Serial.begin(9600);
  
  // Disable all interrupts
  noInterrupts();

  // Configure Timer Interrupt at 200ms
  TCCR1A = 0;
  TCCR1B = 0;
  TCNT1  = 0;
  OCR1A = 50000;                        // Compare match register 16MHz/64/5Hz = 200ms
  TCCR1B |= (1 << WGM12);               // CTC mode
  TCCR1B |= (1 << CS01) | (1 << CS00);  // 64 prescaler 
  TIMSK1 |= (1 << OCIE1A);              // Enable timer compare interrupt

  // Enable all interrupts  
  interrupts();
};

/**
 * Main Program
 */
void loop() {

  // Validate change quantity eggs
  if(qtyEggsActual != qtyEggsPrevious) {

    // Send Report EggBox to PC
    Serial.print("EggBox Report -> Qty Eggs : ");    
    Serial.println(qtyEggsActual);
    
    // Update Quantity Eggs Previous
    qtyEggsPrevious = qtyEggsActual;
  }
}

/**
 * Interrupt Timer Service Routine
 */
ISR(TIMER1_COMPA_vect) 
{
  int sensorState = 0;

  // Clear Quantity Eggs Actual
  qtyEggsActual = 0;

  for(int egg = 0; egg < SENSOR_QTY; egg++) {
  
    // Turn on IR LED of sensor egg + 1
    sensorPowerOn(egg);

    // Wait 1 ms for sensor stabilization 
    delay(1);

    // Read IR sensors pin
    sensorState = digitalRead(SENSORS_INPUT);

    // Validate State Sensor
    if(sensorState == LOW) {

      // Increment Quantity Eggs
      qtyEggsActual++;
    }
    
    // Turn of all IR LED of sensors
    sensorsPowerOff();

    // Sleep microcontroler 1 ms (50% dutty cycle)
    delay(1);
  }
}
  
/**
 * Metod for power on a particular sensor
 *  View Sensors settings array  
 */
void sensorPowerOn(int select) {

  // Select pin mode from A from PIN_CONFIG setting from sensors array
  pinMode(SENSORS_POWER_A, sensors[ select ][ PIN_CONFIG ][ A ]);

  // Select pin mode from B from PIN_CONFIG setting from sensors array
  pinMode(SENSORS_POWER_B, sensors[ select ][ PIN_CONFIG ][ B ]);

  // Select pin mode from C from PIN_CONFIG setting from sensors array
  pinMode(SENSORS_POWER_C, sensors[ select ][ PIN_CONFIG ][ C ]);

  // Select pin mode from D from PIN_CONFIG setting from sensors array
  pinMode(SENSORS_POWER_D, sensors[ select ][ PIN_CONFIG ][ D ]);

  // Select pin state from A from PIN_STATE setting from sensors array
  digitalWrite(SENSORS_POWER_A, sensors[ select ][ PIN_STATE ][ A ]);

  // Select pin state from B from PIN_STATE setting from sensors array
  digitalWrite(SENSORS_POWER_B, sensors[ select ][ PIN_STATE ][ B ]);

  // Select pin state from C from PIN_STATE setting from sensors array
  digitalWrite(SENSORS_POWER_C, sensors[ select ][ PIN_STATE ][ C ]);

  // Select pin state from D from PIN_STATE setting from sensors array
  digitalWrite(SENSORS_POWER_D, sensors[ select ][ PIN_STATE ][ D ]);
};

/**
 * Metod for power off all sensors:
 *  Seto to LOW all sensors power pins
 */

void sensorsPowerOff() {

  digitalWrite(SENSORS_POWER_A, LOW);    // Set SENSORS_POWER_A pin to LOW
  digitalWrite(SENSORS_POWER_B, LOW);    // Set SENSORS_POWER_B pin to LOW
  digitalWrite(SENSORS_POWER_C, LOW);    // Set SENSORS_POWER_C pin to LOW
  digitalWrite(SENSORS_POWER_C, LOW);    // Set SENSORS_POWER_C pin to LOW
};
