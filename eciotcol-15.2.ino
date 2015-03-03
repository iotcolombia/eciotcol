/**
 * @file eceiotcol.ino
 * @brief Eggs Container Embedded IoT Colombia
 * @date 01/03/2015
 * @author Leandro Perez Guatibonza / leandropg AT gmail DOT com
 * @author Giovanni García Rodríguez / giovannigarcia DOT de AT gmail DOT com
 * @author Gabriel García
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

/*
* timer and interrupts
* Timer1 overflow interrupt example
* more infos: http://blog.oscarliang.net
*/

#define LED_PIN 13
#define TimeOnSensor 10

/**
 * Pin Power Sensor
 */
#define SENSOR_POWER 12

/**
 * Pin Input Sensor
 */
#define SENSOR_INPUT 11

#define  COUNT_500MS  25
#define  COUNT_1S  50
#define  COUNT_2S  100
#define  COUNT_5S  250



/*
 * Define Firmware Variables
 */
bool flagSlow = true;  
int count1ms = 0;   // 1 ms counter
int countSlow = 0;    // 
int sensorState;    // Sensor status

void setup()
{
  // Configure output Led pin
  pinMode(LED_PIN, OUTPUT);
  // Configure Input Sensor Pin
  pinMode(SENSOR_INPUT, INPUT_PULLUP);
  // Configure Power Sensor Pin
  pinMode(SENSOR_POWER, OUTPUT);

  // waiting for serial port is enable. Inicially could be a Baud rate of 9600 bps
  // It works very good at 19200, 38400,  57600 and 115200 bps
  Serial.begin(9600);
  while(!Serial){;}

  
  // initialize timer1 
  noInterrupts();           // disable all interrupts
  TCCR1A = 0;
  TCCR1B = 0;
  TCNT1  = 0;

  OCR1A = 2000;            // compare match register 16MHz/8/1KHz
  TCCR1B |= (1 << WGM12);  // CTC mode
  TCCR1B |= (1 << CS11);   // 8 prescaler 
  TIMSK1 |= (1 << OCIE1A); // enable timer compare interrupt
  interrupts();            // enable all interrupts  
}

// interrupt service routine that wraps a user defined function supplied by attachInterrupt
ISR(TIMER1_COMPA_vect) 
{
  // The TIMER1 interrupt vector is enabled to execute this code each 1ms (1KHz)
  // This time base (a.k.a tick) will work to handle the required energy to the IR sensor
  // for a 1 milisecond. The next 19 milisecods, the port will remain shut down.
  // Add to this, the base tick will help to another time-based routines, for example,
  // periodical data  transmition.
  
  if(count1ms < 20)
  {
    count1ms++;  // 1 ms increment 
  }
  else{
    count1ms =0;  // counter reset
    
    // este contador permite prolongar la base de tiempo a un conteo más lento.
    if(countSlow < COUNT_2S)
    {
      countSlow++;
    }
    else{
      countSlow = 0;
      flagSlow = true;  // finish of slow time tick.
    }
  }
}


void loop()
{
// your program here...
  CheckSensor();
  Transmision();

}



void CheckSensor(){
   
   
  if(count1ms < TimeOnSensor)
  {
    digitalWrite(SENSOR_POWER, HIGH); // Turn On IR Led
  }
  else
  {
    sensorState = digitalRead(SENSOR_INPUT); // read IR sensor
    // Validate State Sensor
    if (sensorState == HIGH) {
      digitalWrite(LED_PIN, LOW);     // Turn Off Led
    }
    else {
      digitalWrite(LED_PIN, HIGH);     // Turn On Led
    }
  } 
}

void Transmision(){
  
  if(flagSlow == true)
  {
     if (sensorState == LOW) {
       Serial.println("EGG OK");  // send text with Break line
     }
     else{
       Serial.println("NO EGG");
    }
    flagSlow = false;  // clear the slow tick flag to transmit in the next T seconds
  }
}
