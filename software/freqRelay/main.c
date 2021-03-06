#include <stdio.h>
#include <stdlib.h>
#include "sys/alt_irq.h"
#include "system.h"
#include "io.h"
#include "altera_up_avalon_video_character_buffer_with_dma.h"
#include "altera_up_avalon_video_pixel_buffer_dma.h"
#include <stddef.h>
#include <string.h>
#include <stdbool.h>
#include <stdlib.h>
#include <unistd.h>


#include <altera_avalon_pio_regs.h>

#include "FreeRTOS/FreeRTOS.h"
#include "FreeRTOS/task.h"
#include "FreeRTOS/queue.h"
#include "FreeRTOS/semphr.h"
#include "FreeRTOS/timers.h"

//For frequency plot
#define FREQPLT_ORI_X 101		//x axis pixel position at the plot origin
#define FREQPLT_GRID_SIZE_X 5	//pixel separation in the x axis between two data points
#define FREQPLT_ORI_Y 199.0		//y axis pixel position at the plot origin
#define FREQPLT_FREQ_RES 20.0	//number of pixels per Hz (y axis scale)

#define ROCPLT_ORI_X 101
#define ROCPLT_GRID_SIZE_X 5
#define ROCPLT_ORI_Y 259.0
#define ROCPLT_ROC_RES 0.5		//number of pixels per Hz/s (y axis scale)

#define MIN_FREQ 45.0 //minimum frequency to draw

//define load status
#define UNLOAD 0
#define LOAD 1
#define MANAGED 2

//State declaration
typedef enum{
	NORMAL,
	MONITORING,
	MAINTENANCE
}state;


//Global variables
double freq[100], dfreq[100];
int freqIndex = 99;//freqIndex here points to the oldest data in freq array

double thresholdFreq = 49;
double thresholdROC = 60;

int loadState[5];
int redLED = 0x00;
int greenLED = 0x00;

bool firstLoadSheeding = true;
bool isStable = true;

state systemState = NORMAL;
// Timer handle

TimerHandle_t timer_500;

//Task
#define TASK_STACKSIZE 2048

#define StabilityMonitor_Task_P 7
#define LoadCtrl_Task_P 6
#define switchPolling_Task_P 5
#define LEDCtrl_Task_P 4
#define PRVGADraw_Task_P 2




//Task handler
TaskHandle_t PRVGADraw;


//Queue handler
static QueueHandle_t Q_freq_data;

//Semaphores
xSemaphoreHandle loadStateSemaphore;
xSemaphoreHandle stableSemaphore;
xSemaphoreHandle ledSignal;



/****** Frequency Analyzer ISR ******/
void freq_relay(){
	#define SAMPLING_FREQ 16000.0
	double temp = SAMPLING_FREQ/(double)IORD(FREQUENCY_ANALYSER_BASE, 0);
	xQueueSendToBackFromISR( Q_freq_data, &temp, pdFALSE );
	return;
}
/****** Frequency Analyzer ISR END ******/

/****** Switch Polling Task ******/
void switchPollingTask(void *pvParameters)
{
	while (1)
	{
		int switchState = IORD_ALTERA_AVALON_PIO_DATA(SLIDE_SWITCH_BASE);

		xSemaphoreTake(loadStateSemaphore, portMAX_DELAY);
			for (int i = 0; i < 5; i++)
			{
				if ((switchState & (1 << i)))
				{
					loadState[i] = LOAD;

				}
				else
				{
					loadState[i] = UNLOAD;
				}
			}
		xSemaphoreGive(loadStateSemaphore);
		xSemaphoreGive(ledSignal);
		vTaskDelay(50);
		}

	}
/****** Switch Polling Task END ******/


/****** LED Control Task ******/
void LEDCtrlTask(void *pvParameters) {


	while (1) {

		if(xSemaphoreTake(ledSignal, portMAX_DELAY)){
			redLED = 0x00;
			greenLED = 0x00;

			for (int i = 0; i < 5; i++) {

				if(loadState[i] == LOAD){
					redLED |= (0x1 << i);

			}

			}

		}

		IOWR_ALTERA_AVALON_PIO_DATA(RED_LEDS_BASE, redLED);
		IOWR_ALTERA_AVALON_PIO_DATA(GREEN_LEDS_BASE, greenLED);

		vTaskDelay(50);
	}
}
/****** LED Control Task END ******/


void vTimerCallback(xTimerHandle t_timer500){

}


/****** Stability Monitor Task ******/
void stabilityMonitorTask(void *pvParameters){

	while(1){
		while(uxQueueMessagesWaiting( Q_freq_data ) != 0){
					xQueueReceive( Q_freq_data, freq+freqIndex, 0 );

					//calculate frequency RoC

					if(freqIndex==0){
						dfreq[0] = (freq[0]-freq[99]) * 2.0 * freq[0] * freq[99] / (freq[0]+freq[99]);
					}
					else{
						dfreq[freqIndex] = (freq[freqIndex]-freq[freqIndex-1]) * 2.0 * freq[freqIndex]* freq[freqIndex-1] / (freq[freqIndex]+freq[freqIndex-1]);
					}


					xSemaphoreTake(stableSemaphore, portMAX_DELAY);
					//check the freq&ROC against thresholds
					if ((freq[freqIndex] < thresholdFreq) || (abs(dfreq[freqIndex]) > thresholdROC))
					{

						if(systemState == NORMAL){
							systemState = MONITORING;
							isStable = false;
							//start timer
						}
					}
					else
					{
					   isStable = true;

					}
					xSemaphoreGive(stableSemaphore);

					if (dfreq[freqIndex] > 100.0){
						dfreq[freqIndex] = 100.0;
					}
					freqIndex =	++freqIndex%100; //point to the next data (oldest) to be overwritten(from 0 - 99)
					vTaskDelay(10);
				}
	}
}
/****** Stability Monitor Task END ******/







/****** VGA display Task ******/

typedef struct{
	unsigned int x1;
	unsigned int y1;
	unsigned int x2;
	unsigned int y2;
}Line;

void PRVGADraw_Task(void *pvParameters ){

	//initialize VGA controllers
	alt_up_pixel_buffer_dma_dev *pixel_buf;
	pixel_buf = alt_up_pixel_buffer_dma_open_dev(VIDEO_PIXEL_BUFFER_DMA_NAME);
	if(pixel_buf == NULL){
		printf("can't find pixel buffer device\n");
	}
	alt_up_pixel_buffer_dma_clear_screen(pixel_buf, 0);

	alt_up_char_buffer_dev *char_buf;
	char_buf = alt_up_char_buffer_open_dev("/dev/video_character_buffer_with_dma");
	if(char_buf == NULL){
		printf("can't find char buffer device\n");
	}
	alt_up_char_buffer_clear(char_buf);



	//Set up plot axes
	alt_up_pixel_buffer_dma_draw_hline(pixel_buf, 100, 590, 200, ((0x3ff << 20) + (0x3ff << 10) + (0x3ff)), 0);
	alt_up_pixel_buffer_dma_draw_hline(pixel_buf, 100, 590, 300, ((0x3ff << 20) + (0x3ff << 10) + (0x3ff)), 0);
	alt_up_pixel_buffer_dma_draw_vline(pixel_buf, 100, 50, 200, ((0x3ff << 20) + (0x3ff << 10) + (0x3ff)), 0);
	alt_up_pixel_buffer_dma_draw_vline(pixel_buf, 100, 220, 300, ((0x3ff << 20) + (0x3ff << 10) + (0x3ff)), 0);

	alt_up_char_buffer_string(char_buf, "Frequency(Hz)", 4, 4);
	alt_up_char_buffer_string(char_buf, "52", 10, 7);
	alt_up_char_buffer_string(char_buf, "50", 10, 12);
	alt_up_char_buffer_string(char_buf, "48", 10, 17);
	alt_up_char_buffer_string(char_buf, "46", 10, 22);

	alt_up_char_buffer_string(char_buf, "df/dt(Hz/s)", 4, 26);
	alt_up_char_buffer_string(char_buf, "60", 10, 28);
	alt_up_char_buffer_string(char_buf, "30", 10, 30);
	alt_up_char_buffer_string(char_buf, "0", 10, 32);
	alt_up_char_buffer_string(char_buf, "-30", 9, 34);
	alt_up_char_buffer_string(char_buf, "-60", 9, 36);


	Line line_freq, line_roc;

	while(1){

		//clear old graph to draw new graph
		alt_up_pixel_buffer_dma_draw_box(pixel_buf, 101, 0, 639, 199, 0, 0);
		alt_up_pixel_buffer_dma_draw_box(pixel_buf, 101, 201, 639, 299, 0, 0);

		for(int j=0;j<99;++j){ //i here points to the oldest data, j loops through all the data to be drawn on VGA
			if (((int)(freq[(freqIndex+j)%100]) > MIN_FREQ) && ((int)(freq[(freqIndex+j+1)%100]) > MIN_FREQ)){
				//Calculate coordinates of the two data points to draw a line in between
				//Frequency plot
				line_freq.x1 = FREQPLT_ORI_X + FREQPLT_GRID_SIZE_X * j;
				line_freq.y1 = (int)(FREQPLT_ORI_Y - FREQPLT_FREQ_RES * (freq[(freqIndex+j)%100] - MIN_FREQ));

				line_freq.x2 = FREQPLT_ORI_X + FREQPLT_GRID_SIZE_X * (j + 1);
				line_freq.y2 = (int)(FREQPLT_ORI_Y - FREQPLT_FREQ_RES * (freq[(freqIndex+j+1)%100] - MIN_FREQ));

				//Frequency RoC plot
				line_roc.x1 = ROCPLT_ORI_X + ROCPLT_GRID_SIZE_X * j;
				line_roc.y1 = (int)(ROCPLT_ORI_Y - ROCPLT_ROC_RES * dfreq[(freqIndex+j)%100]);

				line_roc.x2 = ROCPLT_ORI_X + ROCPLT_GRID_SIZE_X * (j + 1);
				line_roc.y2 = (int)(ROCPLT_ORI_Y - ROCPLT_ROC_RES * dfreq[(freqIndex+j+1)%100]);

				//Draw
				alt_up_pixel_buffer_dma_draw_line(pixel_buf, line_freq.x1, line_freq.y1, line_freq.x2, line_freq.y2, 0x3ff << 0, 0);
				alt_up_pixel_buffer_dma_draw_line(pixel_buf, line_roc.x1, line_roc.y1, line_roc.x2, line_roc.y2, 0x3ff << 0, 0);
			}
		}
		vTaskDelay(10);

	}
}
/****** VGA display Task END ******/




int main()
{
	Q_freq_data = xQueueCreate( 100, sizeof(double) );

	loadStateSemaphore= xSemaphoreCreateMutex();
	stableSemaphore = xSemaphoreCreateMutex();
	vSemaphoreCreateBinary(ledSignal);

	timer_500 = xTimerCreate("Timer 500", 500 , pdTRUE, NULL, vTimerCallback);


	alt_irq_register(FREQUENCY_ANALYSER_IRQ, 0, freq_relay);

	xTaskCreate( PRVGADraw_Task, "DrawTsk", TASK_STACKSIZE, NULL, PRVGADraw_Task_P, &PRVGADraw );
	xTaskCreate( stabilityMonitorTask, "StabilityMontitorTsk", TASK_STACKSIZE, NULL, StabilityMonitor_Task_P, NULL);
	xTaskCreate( switchPollingTask, "SwitchPollingTsk", TASK_STACKSIZE, NULL, switchPolling_Task_P, NULL);
	xTaskCreate( LEDCtrlTask, "LEDCtrlTsk", TASK_STACKSIZE, NULL, LEDCtrl_Task_P, NULL);



	vTaskStartScheduler();

	while(1)

  return 0;
}
