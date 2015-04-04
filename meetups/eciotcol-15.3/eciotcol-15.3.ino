/**
 * @file eceiotcol.ino
 * @brief Eggs Container Embedded IoT Colombia
 * @date 04/04/2015
 * @author Leandro Perez Guatibonza / leandropg AT gmail DOT com
 * @author Giovanni García Rodríguez / giovannigarcia DOT de AT gmail DOT com
 * @author Gabriel García
 * @author Jorge Ernesto Guevara Cuneca / guevara DOT ernesto AT gmail DOT com
 * @version 15.3
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

#define LED_PIN          13   // Egg indicator pin
#define SENSORS_INPUT    11   // Sensors inputs pin (Transistors colectors)
#define SENS_POW_A       10   // Sensor power pin (Charlieplexing - Diodes emitters)
#define SENS_POW_B       9    // Sensor power pin (Charlieplexing - Diodes emitters)
#define SENS_POW_C       8    // Sensor power pin (Charlieplexing - Diodes emitters)
#define SENS_POW_D       7    // Sensor power pin (Charlieplexing - Diodes emitters)

#define CHARLIEPLEXING   4    // Multiplexing number of pins
#define SENSOR_COUNT     12   // Multiplexing number of sensors

#define PIN_CONFIG       0    // Pin configuration selector sensors array
#define PIN_STATE        1    // Pin state selector sensors array
#define A                0    // A selector sensors array
#define B                1    // B selector sensors array
#define C                2    // C selector sensors array
#define D                3    // D selector sensors array

/**
 * Sensor state
 */
int sensorState = 0;

/**
 * Loop counter
 */
int egg = 0;

/**
 * Sensors settings array:
 *  Set 2 pins to OUTPUT and set only one of them to HIGH to power on a sensor at time 
 */
int sensors[ SENSOR_COUNT ][ 2 ][ CHARLIEPLEXING ] = {
  //    PIN_CONFIG                           PIN_STATE
  //    A       B       C      D          A    B    C    D
  { { INPUT, OUTPUT, INPUT, OUTPUT }, { LOW, LOW, LOW, HIGH } },  // SENSOR 1 ( SENS_POW_D SENS_POW_B )
  { { INPUT, OUTPUT, INPUT, OUTPUT }, { LOW, HIGH, LOW, LOW } },  // SENSOR 2 ( SENS_POW_B SENS_POW_D )
  { { INPUT, INPUT, OUTPUT, OUTPUT }, { LOW, LOW, LOW, HIGH } },  // SENSOR 3 ( SENS_POW_D SENS_POW_C )
  { { INPUT, INPUT, OUTPUT, OUTPUT }, { LOW, LOW, HIGH, LOW } },  // SENSOR 4 ( SENS_POW_C SENS_POW_D )
  { { INPUT, OUTPUT, OUTPUT, INPUT }, { LOW, LOW, HIGH, LOW } },  // SENSOR 5 ( SENS_POW_C SENS_POW_B )
  { { INPUT, OUTPUT, OUTPUT, INPUT }, { LOW, HIGH, LOW, LOW } },  // SENSOR 6 ( SENS_POW_B SENS_POW_C )  
  { { OUTPUT, INPUT, INPUT, OUTPUT }, { LOW, LOW, LOW, HIGH } },  // SENSOR 7 ( SENS_POW_D SENS_POW_A )
  { { OUTPUT, INPUT, INPUT, OUTPUT }, { HIGH, LOW, LOW, LOW } },  // SENSOR 8 ( SENS_POW_A SENS_POW_D )
  { { OUTPUT, INPUT, OUTPUT, INPUT }, { LOW, LOW, HIGH, LOW } },  // SENSOR 9 ( SENS_POW_C SENS_POW_A )
  { { OUTPUT, INPUT, OUTPUT, INPUT }, { HIGH, LOW, LOW, LOW } },  // SENSOR 10 ( SENS_POW_A SENS_POW_C )
  { { OUTPUT, OUTPUT, INPUT, INPUT }, { LOW, HIGH, LOW, LOW } },  // SENSOR 11 ( SENS_POW_B SENS_POW_A )
  { { OUTPUT, OUTPUT, INPUT, INPUT }, { HIGH, LOW, LOW, LOW } },  // SENSOR 12 ( SENS_POW_A SENS_POW_B )
};

 /**
 * Init Routine
 */
void setup() {
  
  // Configure output egg indicator LED pin
  pinMode( LED_PIN, OUTPUT );
  
  // Configure inputs sensors pin
  pinMode( SENSORS_INPUT, INPUT_PULLUP );
  
  Serial.begin( 9600 );
};

/**
 * Main Program
 */
void loop() {
  
  if( egg < SENSOR_COUNT ) {
    
    sensorPowerOn( egg );                        // Turn on IR LED of sensor egg+1
    
    delay( 1 );                                  // Wait 1 ms for sensor stabilization 
    
    sensorState = digitalRead( SENSORS_INPUT );  // Read IR sensors pin
    
    // Validate State Sensor
    if( sensorState == HIGH ) {
      
      digitalWrite( LED_PIN, LOW );              // Turn off egg indicator LED
    }
    else {
      
      digitalWrite( LED_PIN, HIGH );             // Turn on egg indicator LED
      
      Serial.print( "EGG " );                    // Transmit the number of sensor
      Serial.print( egg + 1 );                   //  when an egg is on sensor
      Serial.println( " OK" );                   //  EGG egg+1 OK
    }
    
    sensorsPowerOff();                           // Turn of all IR LED of sensors
    
    delay( 1 );                                  // Sleep microcontroler 1 ms (50% dutty cycle)
    
    egg++;
  }
  else {
    egg = 0;
  }
}

/**
 * Metod for power on a particular sensor
 *  View Sensors settings array  
 */
void sensorPowerOn( int select ) {
  
  // Select pin mode from A from PIN_CONFIG setting from sensors array
  pinMode( SENS_POW_A, sensors[ select ][ PIN_CONFIG ][ A ] );
  
  // Select pin mode from B from PIN_CONFIG setting from sensors array
  pinMode( SENS_POW_B, sensors[ select ][ PIN_CONFIG ][ B ] );
  
  // Select pin mode from C from PIN_CONFIG setting from sensors array
  pinMode( SENS_POW_C, sensors[ select ][ PIN_CONFIG ][ C ] );
  
  // Select pin mode from D from PIN_CONFIG setting from sensors array
  pinMode( SENS_POW_D, sensors[ select ][ PIN_CONFIG ][ D ] );
  
  // Select pin state from A from PIN_STATE setting from sensors array
  digitalWrite( SENS_POW_A, sensors[ select ][ PIN_STATE ][ A ] );
  
  // Select pin state from B from PIN_STATE setting from sensors array
  digitalWrite( SENS_POW_B, sensors[ select ][ PIN_STATE ][ B ] );
  
  // Select pin state from C from PIN_STATE setting from sensors array
  digitalWrite( SENS_POW_C, sensors[ select ][ PIN_STATE ][ C ] );
  
  // Select pin state from D from PIN_STATE setting from sensors array
  digitalWrite( SENS_POW_D, sensors[ select ][ PIN_STATE ][ D ] );
};

/**
 * Metod for power off all sensors:
 *  Seto to LOW all sensors power pins
 */
void sensorsPowerOff() {
  
  digitalWrite( SENS_POW_A, LOW );  // Set SENS_POW_A pin to LOW
  
  digitalWrite( SENS_POW_B, LOW );  // Set SENS_POW_B pin to LOW

  digitalWrite( SENS_POW_C, LOW );  // Set SENS_POW_C pin to LOW
  
  digitalWrite( SENS_POW_C, LOW );  // Set SENS_POW_C pin to LOW
};
