/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * Author: Louis Mott                                                                      *
 * Purpose: Thermal Alarm                                                                  *
 * Assignment: Project 3                                                                   *
 *****************************************************                                     *
 * Inputs:  PD9                                                                            *
 * Outputs: PB8, PB9, PC0, PC1, PE9                                                        *
 *****************************************************                                     *
 * References:                                                                             *  
 *    - https://os.mbed.com/users/fossum_13/code/DHT11/docs/5da6f6de3e42/classDht11.html   * 
 *    - Nov. 13 Lecture                                                                                                              VCC*  
 * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

#include "Dht11.h"
#include "1802.h"
#include "Thread.h"
#include "mbed.h"
#include <cstdint>
#include <cstdio>
#include <string>

void checkTemp(void);
void danger();

const int threshold = 19;   // 100F == 37.7C
const int safe = 0;         // 0 == SAFE
const int dngr = 1;       // 1 == DANGER
const int speed = 1000;

int temp = 0;               // keeps track of temperature.
char pTemp[5];              // lcd version of temp (5 char buffer -999 to 9999 + \0)
int state = safe;           // keeps track of safe/danger state. (Default==safe)

// LCD
CSE321_LCD lcd(16, 2, LCD_5x8DOTS, PB_9, PB_8);

// Pulse With Modulation 
// PE_9 PWM Output port
PwmOut sound(PE_9);

// Temperature and Humidity Sensor
Dht11 sensor(PD_9);
// Create Watchdog
// Watchdog &watchMe = Watchdog::get_instance();

// setup interrupt objects
InterruptIn int1(PC_0);
InterruptIn int2(PC_1);



/*
 * Pulse Buzzer Module
 */

void danger(){
    sound.write(0.5f);          // setting 50% DC
    int per = 10000;            // 20000us
    sound.period_us(per);       // Update period
}

/*
 * If temperature is above threshold
 * Change state to DANGER and sounds alarm
 * Else change state to SAFE
 */

void checkTemp(void) {
    sensor.read();
    temp = sensor.getCelsius();
    sprintf(pTemp, "%d", temp); // convert int to char buffer(https://devdocs.io/c/io/fprintf)
    printf("T: %d\r\n", temp);

    if(threshold < temp){
      state = dngr;
      danger();
    }
    else{
      state = safe;
    }
    wait_us(500); // 500 us
}


int main() {


  lcd.begin();
    

  // RCC & MODDER
  RCC->AHB2ENR |= 7;
  GPIOB->MODER &= ~(0xF0000);

  // GPIOC OUTPUTS
  GPIOC->MODER &= ~(0xA);
  GPIOC->MODER |= 0x5;

  //GPIOB INPUTS
  GPIOD->MODER &= ~(0xC0000);
  GPIOD->MODER |= 0x60000;

  int1.rise(&checkTemp);
  int2.rise(&checkTemp);
  // enable
  int1.enable_irq();
  int2.enable_irq();

  while (true) {

    // sensor.read();
    // temp = sensor.getCelsius();

    // sprintf(pTemp, "%d", temp); // convert int to char buffer(https://www.geeksforgeeks.org/sprintf-in-c/)
    // printf("T: %d\r\n", temp);
 
    // delay
    if (state == safe) {
      lcd.clear();
      lcd.print("Safe: ");
      lcd.print(pTemp);
      GPIOC->ODR &= ~0x1;   //trun off green led
      GPIOC->ODR &= ~0x2;   //turn off red led
      GPIOC->ODR |= 0x1;    //turn on green led
    } else if (state == dngr) {
      lcd.clear();
      lcd.print("Danger: ");
      lcd.print(pTemp);
      danger();
      GPIOC->ODR &= ~0x2;   //turn off red led
      GPIOC->ODR &= ~0x1;   //trun off green led
      GPIOC->ODR |= 0x2;    //turn on red led
      thread_sleep_for(1000);
      GPIOC->ODR &= ~0x2;   //turn off red led
    }
    thread_sleep_for(50);
  }
}