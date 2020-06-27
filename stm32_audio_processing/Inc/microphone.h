#ifndef _MICROPHONE_H_
#define _MICROPHONE_H_

#include "main.h"
#include "cmsis_os.h"
#include "stdbool.h"

#define I2S2                            SPI2

/* Audio status definition */     
#define AUDIO_OK                        0
#define AUDIO_ERROR                     1
#define AUDIO_TIMEOUT                   2

#define I2S2_CLK_ENABLE()               __HAL_RCC_SPI2_CLK_ENABLE()
#define I2S2_CLK_DISABLE()              __HAL_RCC_SPI2_CLK_DISABLE()
#define I2S2_SCK_PIN                    GPIO_PIN_10
#define I2S2_SCK_GPIO_PORT              GPIOB
#define I2S2_SCK_GPIO_CLK_ENABLE()      __HAL_RCC_GPIOB_CLK_ENABLE()
#define I2S2_SCK_AF                     GPIO_AF5_SPI2

#define I2S2_MOSI_PIN                   GPIO_PIN_3
#define I2S2_MOSI_GPIO_PORT             GPIOC
#define I2S2_MOSI_GPIO_CLK_ENABLE()     __HAL_RCC_GPIOC_CLK_ENABLE()
#define I2S2_MOSI_AF                    GPIO_AF5_SPI2

#define I2S2_DMAx_CLK_ENABLE()          __HAL_RCC_DMA1_CLK_ENABLE()
#define I2S2_DMAx_CLK_DISABLE()         __HAL_RCC_DMA1_CLK_DISABLE()
#define I2S2_DMAx_STREAM                DMA1_Stream3
#define I2S2_DMAx_CHANNEL               DMA_CHANNEL_0
#define I2S2_DMAx_IRQ                   DMA1_Stream3_IRQn
#define I2S2_DMAx_PERIPH_DATA_SIZE      DMA_PDATAALIGN_HALFWORD
#define I2S2_DMAx_MEM_DATA_SIZE         DMA_MDATAALIGN_HALFWORD
   
#define I2S2_IRQHandler                 DMA1_Stream3_IRQHandler

#define AUDIO_IN_IRQ_PREPRIO            0x0F   /* Select the preemption priority level(0 is the highest) */

#define DEFAULT_AUDIO_IN_FREQ                 I2S_AUDIOFREQ_8K
#define DEFAULT_AUDIO_IN_BIT_RESOLUTION       16
#define DEFAULT_AUDIO_IN_CHANNEL_NBR          1 /* Mono = 1, Stereo = 2 */

/* PDM buffer input size */
#define INTERNAL_BUFF_SIZE                    128
/* PCM buffer output size */
#define PCM_OUT_SIZE                          DEFAULT_AUDIO_IN_FREQ/1000

#define WR_SIZE 															4096

#define HTONS(A)  ((((uint16_t)(A) & 0xff00) >> 8) | (((uint16_t)(A) & 0x00ff) << 8))


struct AudioIn
{
	osSemaphoreId s_audio;
	osMutexId			m_audio;
	
	xQueueHandle q_audioToFFT;

	
	osThreadId 		audioInTaskHandle;
	void (*ptrTask)(void const * );
	
	bool open;
	
	struct
	{
			I2S_HandleTypeDef   hAudioIn;
			uint32_t AudioFreq;
			uint32_t BitRes;
			uint32_t ChnlNbr;
			uint32_t ChnlNbrIn;
			uint32_t ChnlNbrOut;
	}init;
	
	struct 
	{
		uint64_t packet_cnt;
		uint64_t queue_error;
	}statistic;
};

void AUDIO_IN_ClockConfig();
void AUDIO_IN_PDMDecoder_Init(struct AudioIn * p);
uint8_t AUDIO_IN_I2S2_Init(struct AudioIn * p);
struct AudioIn * AUDIO_IN_Init();
void AUDIO_IN_MspInit(I2S_HandleTypeDef *hi2s);
void AUDIO_IN_Record(struct AudioIn * p,uint16_t* pbuf, uint32_t size);
uint8_t AUDIO_IN_PDMToPCM(uint16_t *PDMBuf, uint16_t *PCMBuf);

#endif