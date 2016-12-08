/*
 *     SocialLedge.com - Copyright (C) 2013
 *
 *     This file is part of free software framework for embedded processors.
 *     You can use it and/or distribute it as long as this copyright header
 *     remains unmodified.  The code is free for personal use and requires
 *     permission to use in a commercial product.
 *
 *      THIS SOFTWARE IS PROVIDED "AS IS".  NO WARRANTIES, WHETHER EXPRESS, IMPLIED
 *      OR STATUTORY, INCLUDING, BUT NOT LIMITED TO, IMPLIED WARRANTIES OF
 *      MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE APPLY TO THIS SOFTWARE.
 *      I SHALL NOT, IN ANY CIRCUMSTANCES, BE LIABLE FOR SPECIAL, INCIDENTAL, OR
 *      CONSEQUENTIAL DAMAGES, FOR ANY REASON WHATSOEVER.
 *
 *     You can reach the author of this software at :
 *          p r e e t . w i k i @ g m a i l . c o m
 */

/**
 * @file
 * @brief This is the application entry point.
 * 			FreeRTOS and stdio printf is pre-configured to use uart0_min.h before main() enters.
 * 			@see L0_LowLevel/lpc_sys.h if you wish to override printf/scanf functions.
 *
 */
#include "tasks.hpp"
#include "examples/examples.hpp"
#include "stdio.h"
#include "adc0.h"
#include "stdlib.h"
#include "math.h"

/*
 * Voltage Levels:
 * 3.3v max (100 %)
 * 2.97 - 90 %			if under, start charging automatically
 * 2.64 - 80 %
 * 2.31 - 70 %
 * 1.98 - 60 %
 * 1.65 - 50 %
 * 1.32 - 40 %
 * 0.99 - 30 %
 * 0.66 - 20 %
 * 0.33 - 10 %			if under, can no longer use motor; must retain final 10 % to operate board
 */

float batteryV = 0;
float batteryPct = 0;
int samplesTaken = 0, debug = 0;;

class adc0_task : public scheduler_task {
	public:
		adc0_task(uint8_t priority): scheduler_task("adc", 2000, priority)
		{}
		bool run(void *p){
			vTaskDelay(1);

			//use pre-defined adc0.h class
			batteryV += adc0_get_reading(4) * 0.000805;
			samplesTaken++;

			if(samplesTaken % 500 == 0){
				batteryV /= 500;
				batteryPct = batteryV / .033;
				batteryV = 0;
			}

			if(samplesTaken % 1000 == 0){
				printf("DEBUG - ADC0: Battery Level = %f\n", batteryPct);
				samplesTaken = 0;
			}

			return true;
		}
		bool init(void){
			//adc0 already initialized at startup
			LPC_PINCON->PINSEL3 |= (3<<28); //configure p1.30 as adc (ad0.4)
			return true;
		}
};

class mosfet_task : public scheduler_task {
	public:
		mosfet_task(uint8_t priority): scheduler_task("mosfet", 2000, priority)
		{}
		bool run(void *p){
			vTaskDelay(10);
			debug++;

			if(batteryPct < 90){ //if battery is less than 90 %, charge
				LPC_GPIO2->FIOCLR = (1<<1); //activate mosfet at p2.1 (generator)
				if(debug % 100 == 0){
					debug = 0;
					printf("DEBUG - MOSFET: Generator Activated\n");
				}
			}
			else if(batteryPct > 98){
				LPC_GPIO2->FIOSET = (1<<1); //deactivate mosfet at p2.1 (generator)
				if(debug % 100 == 0){
					debug = 0;
					printf("DEBUG - MOSFET: Generator Deactivated\n");
				}
			}

			if(batteryPct < 10){ //if battery is under 10 %, deactivate motor
				LPC_GPIO2->FIOSET = (1<<3); //deactivate mosfet at p2.3 (motor)
				if(debug % 100 == 0){
					debug = 0;
					printf("DEBUG - MOSFET: Motor Deactivated\n");
				}
			}
			else if(LPC_GPIO1->FIOPIN &(1<<15)){ //if battery is greater than 10 % and button pressed, activate motor
				LPC_GPIO2->FIOCLR = (1<<3); //activate mosfet at p2.3 (motor)
				if(debug % 100 == 0){
					debug = 0;
					printf("DEBUG - MOSFET: Motor Activated\n");
				}
			}
			else{ //button has not been pressed
				LPC_GPIO2->FIOSET = (1<<3); //deactivate mosfet at p2.3 (motor)
				if(debug % 100 == 0){
					debug = 0;
					printf("DEBUG - MOSFET: Motor Not Active\n");
				}
			}

			return true;
		}
		bool init(void){
			LPC_PINCON->PINSEL2 &= ~(3<<0); //configure p1.15 as gpio

			LPC_PINCON->PINSEL4 &= ~(3<<2); //configure p2.1 as gpio
			LPC_PINCON->PINSEL4 &= ~(3<<6); //configure p2.3 as gpio
			LPC_GPIO2->FIODIR |= (1<<1); //initialize p2.1 as output (generator)
			LPC_GPIO2->FIODIR |= (1<<3); //initialize p2.3 as output (motor)
			return true;
		}
};

void uart2_init(uint32_t baud){
	LPC_PINCON->PINSEL4 &= ~(15<<16); //clear p2.8 as txd2 and p2.9 as rxd2
	LPC_PINCON->PINSEL4 |= (10<<16); //set p2.8 as txd2 and p2.9 as rxd2
	LPC_SC->PCONP |= (1<<24); //power
	LPC_SC->PCLKSEL1 &= ~(3<<16); //clear uart2 clock msb for div 1
	LPC_SC->PCLKSEL1 |= (1<<16); //enable uart2 clock div 1
	LPC_UART2->LCR |= 3; //set data size as a byte
	LPC_UART2->LCR |= (1<<7); //enable DLAB bit
	uint16_t div = (48*1000*1000)/ (16*baud); //calculate speed value
	LPC_UART2->DLL = div;
	LPC_UART2->DLM = (div>>8);
	LPC_UART2->LCR &= ~(1<<7); //disable DLAB bit
}

void uart2_putchar(char out){
	LPC_UART2->THR = out; //store data in transmission buffer to be sent
	while(1){
		if (LPC_UART2->LSR & (1<<5)) //wait until data is sent
			break;
	}
	printf("DEBUG - UART: putchar sent %c = %x\n", out, out);
}

void uart2_end(){ //communication end from screen datasheet
	uart2_putchar(0xFF);
	uart2_putchar(0xFF);
	uart2_putchar(0xFF);
}

void screenSlider(int value){
			value = (value == 100)? 99: value;
			char num [2];
			sprintf(num, "%d", value);
			uart2_putchar('j');
			uart2_putchar('0');
			uart2_putchar('.');
			uart2_putchar('v');
			uart2_putchar('a');
			uart2_putchar('l');
			uart2_putchar('=');
			uart2_putchar(num[0]);
			if(value > 9)
				uart2_putchar(num[1]);
			uart2_end();
			printf("DEBUG - SCREEN: Slider sent value: %i\n", value);
}

void screenTextPct(int value){
			value = (value == 100)? 99: value;
			char num [2];
			sprintf(num, "%d", value);
			uart2_putchar('t');
			uart2_putchar('1');
			uart2_putchar('.');
			uart2_putchar('t');
			uart2_putchar('x');
			uart2_putchar('t');
			uart2_putchar('=');
			uart2_putchar('"');
			uart2_putchar(num[0]);
			if(value > 9)
				uart2_putchar(num[1]);
			uart2_putchar('"');
			uart2_end();
			printf("DEBUG - SCREEN: Txt pct sent value: %i\n", value);
}

void screenTextV(float value){
			value = (value >= 100)? 99.99: value;
			char num [5];
			sprintf(num, "%f", value);
			uart2_putchar('t');
			uart2_putchar('0');
			uart2_putchar('.');
			uart2_putchar('t');
			uart2_putchar('x');
			uart2_putchar('t');
			uart2_putchar('=');
			uart2_putchar('"');
			uart2_putchar(num[0]);
			uart2_putchar(num[1]);
			uart2_putchar(num[2]);
			uart2_putchar(num[3]);
			uart2_putchar(num[4]);
			uart2_putchar('"');
			uart2_end();
			printf("DEBUG - SCREEN: Txt V sent value: %f\n", value);
}

class uart2_send_task : public scheduler_task {
	public:
	uart2_send_task(uint8_t priority): scheduler_task("uart2 send", 2000, priority)
	{}
	bool run(void *p){
		vTaskDelay(1000);
		int valuePct = int(batteryPct);
		float valueV = batteryPct * .033;
		valueV = roundf(valueV * 100) / 100;
		screenTextV(valueV);
		screenTextPct(valuePct);
		screenSlider(valuePct);
		return true;
	}
	bool init(void){
	    uart2_init(38400);
		return true;
	}
};

/**
 * The main() creates tasks or "threads".  See the documentation of scheduler_task class at scheduler_task.hpp
 * for details.  There is a very simple example towards the beginning of this class's declaration.
 *
 * @warning SPI #1 bus usage notes (interfaced to SD & Flash):
 *      - You can read/write files from multiple tasks because it automatically goes through SPI semaphore.
 *      - If you are going to use the SPI Bus in a FreeRTOS task, you need to use the API at L4_IO/fat/spi_sem.h
 *
 * @warning SPI #0 usage notes (Nordic wireless)
 *      - This bus is more tricky to use because if FreeRTOS is not running, the RIT interrupt may use the bus.
 *      - If FreeRTOS is running, then wireless task may use it.
 *        In either case, you should avoid using this bus or interfacing to external components because
 *        there is no semaphore configured for this bus and it should be used exclusively by nordic wireless.
 */
int main(void)
{
    /**
     * A few basic tasks for this bare-bone system :
     *      1.  Terminal task provides gateway to interact with the board through UART terminal.
     *      2.  Remote task allows you to use remote control to interact with the board.
     *      3.  Wireless task responsible to receive, retry, and handle mesh network.
     *
     * Disable remote task if you are not using it.  Also, it needs SYS_CFG_ENABLE_TLM
     * such that it can save remote control codes to non-volatile memory.  IR remote
     * control codes can be learned by typing the "learn" terminal command.
     */

    scheduler_add_task(new adc0_task(PRIORITY_HIGH));
//    scheduler_add_task(new mosfet_task(PRIORITY_HIGH));
    scheduler_add_task(new uart2_send_task(PRIORITY_HIGH));

	scheduler_add_task(new terminalTask(PRIORITY_HIGH));

    /* Consumes very little CPU, but need highest priority to handle mesh network ACKs */
    scheduler_add_task(new wirelessTask(PRIORITY_CRITICAL));

    /* Change "#if 0" to "#if 1" to run period tasks; @see period_callbacks.cpp */
    #if 0
    scheduler_add_task(new periodicSchedulerTask());
    #endif

    /* The task for the IR receiver */
    // scheduler_add_task(new remoteTask  (PRIORITY_LOW));

    /* Your tasks should probably used PRIORITY_MEDIUM or PRIORITY_LOW because you want the terminal
     * task to always be responsive so you can poke around in case something goes wrong.
     */

    /**
     * This is a the board demonstration task that can be used to test the board.
     * This also shows you how to send a wireless packets to other boards.
     */
    #if 0
        scheduler_add_task(new example_io_demo());
    #endif

    /**
     * Change "#if 0" to "#if 1" to enable examples.
     * Try these examples one at a time.
     */
    #if 0
        scheduler_add_task(new example_task());
        scheduler_add_task(new example_alarm());
        scheduler_add_task(new example_logger_qset());
        scheduler_add_task(new example_nv_vars());
    #endif

    /**
	 * Try the rx / tx tasks together to see how they queue data to each other.
	 */
    #if 0
        scheduler_add_task(new queue_tx());
        scheduler_add_task(new queue_rx());
    #endif

    /**
     * Another example of shared handles and producer/consumer using a queue.
     * In this example, producer will produce as fast as the consumer can consume.
     */
    #if 0
        scheduler_add_task(new producer());
        scheduler_add_task(new consumer());
    #endif

    /**
     * If you have RN-XV on your board, you can connect to Wifi using this task.
     * This does two things for us:
     *   1.  The task allows us to perform HTTP web requests (@see wifiTask)
     *   2.  Terminal task can accept commands from TCP/IP through Wifly module.
     *
     * To add terminal command channel, add this at terminal.cpp :: taskEntry() function:
     * @code
     *     // Assuming Wifly is on Uart3
     *     addCommandChannel(Uart3::getInstance(), false);
     * @endcode
     */
    #if 0
        Uart3 &u3 = Uart3::getInstance();
        u3.init(WIFI_BAUD_RATE, WIFI_RXQ_SIZE, WIFI_TXQ_SIZE);
        scheduler_add_task(new wifiTask(Uart3::getInstance(), PRIORITY_LOW));
    #endif

    scheduler_start(); ///< This shouldn't return
    return -1;
}
