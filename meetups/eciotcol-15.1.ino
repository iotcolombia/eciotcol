/**
 * @file eceiotcol.ino
 * @brief Eggs Container Embedded IoT Colombia
 * @date 08/02/2015
 * @author Leandro Perez Guatibonza / leandropg@gmail.com
 * @version 15.1
 *
 * Container Eggs Embedded IoT Colombia
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

/**
 * Pin Led
 */
int LED = 13;

/**
 * Pin Power Sensor
 */
int SENSOR_POWER = 12;

/**
 * Pin Input Sensor
 */
int SENSOR_INPUT = 11;

/*
 * Define Sensor State Variable
 */
int sensorState;

/**
 * Init Routine
 */
void setup() {

    // Configure Input Sensor Pin
    pinMode(SENSOR_INPUT, INPUT_PULLUP);

    // Configure Power Sensor Pin
    pinMode(SENSOR_POWER, OUTPUT);
    
    // Configure Led Pin
    pinMode(LED, OUTPUT);
}

/**
 * Main Program
 */
void loop() {

    // Turn On Infrared Sensor
    digitalWrite(SENSOR_POWER, HIGH);

    // Wait 1 ms for Stabilization Sensor
    delay(1);

    // Read Sensor
    sensorState = digitalRead(SENSOR_INPUT);

    // Validate State Sensor
    if(sensorState == HIGH) {

        // Turn Off Led
        digitalWrite(LED, LOW);
    }
    else {

        // Turn On Led
        digitalWrite(LED, HIGH);
    }

    // Turn Off Infrared Sensor
    digitalWrite(SENSOR_POWER, LOW);      // Apaga Sensor Infrarrojo

    // Sleep Microcontroller 19 ms
    delay(19);
}

