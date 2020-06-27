#include  "processing.h"
#include  "string.h"
#include  "string.h"
#include <math.h>
#include "arm_math.h"
#include "microphone.h"
#include "stdio.h"

#define FFT_Length_Tab 1024
#define FFT_INVERSE_FLAG        ((uint8_t)0)
#define FFT_Normal_OUTPUT_FLAG  ((uint8_t)1)
#define US_IN_SECOND  					((uint32_t)1000000)

q15_t aFFT_Input_Q15[FFT_Length_Tab*2];
q15_t FFT_Output_Q15[FFT_Length_Tab];
uint32_t nb_cycles        = 0x00,duration_us=0;
q15_t maxValue;    /* Max FFT value is stored here */
uint32_t maxIndex;    /* Index in Output array where max value is */

float32_t katsayi = 7.8125;
float32_t res;

void signalProcTask(void const * argument) 
{
	ts_audio * priv = (ts_audio *)argument;	
  arm_cfft_radix4_instance_q15  FFT_Q15_struct;

	uint16_t *audBuf;
	
	arm_cfft_radix4_init_q15(&FFT_Q15_struct, FFT_Length_Tab, FFT_INVERSE_FLAG, FFT_Normal_OUTPUT_FLAG);

	for(;;)
	{
		if(xQueueReceive(priv->t->q_audioToFFT, &audBuf, 1000) == pdPASS) 
		{		
			if(priv->t->open == true)
			{
				HAL_GPIO_TogglePin(GPIOD, LD6_Pin); 
				//fft analizim burada yapilacaktir
				//toplam 2048 sample geliyor
				//sample[0] = ses 
				//sample[1] = 0
				
				// copy audio data to perform FFT (2048 words / 4096 bytes)
				arm_copy_q15((q15_t *)(audBuf), aFFT_Input_Q15, FFT_Length_Tab*2);
				// perform FFT on 512 real inputs (512 imaginary inputs are 0)
				TimerCount_Start();
				arm_cfft_radix4_q15(&FFT_Q15_struct, aFFT_Input_Q15);
				/* Process the data through the Complex Magniture Module for calculating the magnitude at each bin */
				arm_cmplx_mag_q15(aFFT_Input_Q15, FFT_Output_Q15, FFT_Length_Tab);
				arm_max_q15(FFT_Output_Q15, FFT_Length_Tab, &maxValue, &maxIndex);
				
				res = katsayi*maxIndex;
				
				printf("FREQUENCY : %f \r\n",res);

				TimerCount_Stop(nb_cycles);
				duration_us = (uint32_t)(((uint64_t)US_IN_SECOND * (nb_cycles)) / SystemCoreClock);

			}
		}
	}
}

struct proccesing* proccesing_init(void * p)
{
	struct proccesing * sig = (struct proccesing*)pvPortMalloc(sizeof(struct proccesing));

	sig->ptrTask = signalProcTask;
	
	osThreadDef(procTask, sig->ptrTask, osPriorityNormal, 0, 128);
  sig->proccesingTaskHandle = osThreadCreate(osThread(procTask),(ts_audio*)p);
}