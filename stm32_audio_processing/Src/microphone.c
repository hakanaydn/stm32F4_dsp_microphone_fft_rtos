#include "microphone.h"
#include "..\Middlewares\STM32_Audio\Addons\PDM\Inc\pdm2pcm_glo.h"
#include "string.h"

PDM_Filter_Handler_t  PDM_FilterHandler[2];
PDM_Filter_Config_t   PDM_FilterConfig[2];

extern ts_audio sAudio;

static uint16_t InternalBuffer[INTERNAL_BUFF_SIZE];//64 sample
static uint16_t RecBuf[PCM_OUT_SIZE*2];//8 sample alir fakat 16 sample olur right ve left olarak

uint16_t WrBuffer[WR_SIZE];

__IO uint32_t ITCounter = 0;
__IO uint32_t AudioBuffOffset = 0;

/**
  * @brief  
  * @param  argument: 
  * @retval None
  */
void AUDIO_IN_ClockConfig()
{
  RCC_PeriphCLKInitTypeDef rccclkinit;
	
  HAL_RCCEx_GetPeriphCLKConfig(&rccclkinit);
	
	rccclkinit.PeriphClockSelection = RCC_PERIPHCLK_I2S;
	rccclkinit.PLLI2S.PLLI2SN = 256;//192;
	rccclkinit.PLLI2S.PLLI2SR = 30;//6;
	HAL_RCCEx_PeriphCLKConfig(&rccclkinit);
}

/**
  * @brief  Converts audio format from PDM to PCM.
  * @param  PDMBuf: Pointer to data PDM buffer
  * @param  PCMBuf: Pointer to data PCM buffer
  * @retval AUDIO_OK if correct communication, else wrong communication
  */
uint8_t AUDIO_IN_PDMToPCM(uint16_t *PDMBuf, uint16_t *PCMBuf)
{
  uint16_t AppPDM[INTERNAL_BUFF_SIZE/2];
  uint32_t index = 0; 
  
  /* PDM Demux */
  for(index = 0; index<INTERNAL_BUFF_SIZE/2; index++)
  {
    AppPDM[index] = HTONS(PDMBuf[index]);
  }
  
  for(index = 0; index < DEFAULT_AUDIO_IN_CHANNEL_NBR; index++)
  {
    /* PDM to PCM filter */
		PDM_Filter((uint8_t*)&AppPDM[index], (uint16_t*)&(PCMBuf[index]), &PDM_FilterHandler[index]);
  }
  /* Duplicate samples since a single microphone in mounted on STM32F4-Discovery */
  /*for(index = 0; index < PCM_OUT_SIZE; index++)
  {
    PCMBuf[(index<<1)+1] = PCMBuf[index<<1];
  }*/
  
  /* Return AUDIO_OK when all operations are correctly done */
  return AUDIO_OK; 
}

/**
  * @brief  
  * @param  argument: 
  * @retval None
  */
void AUDIO_IN_PDMDecoder_Init(struct AudioIn * p)
{
  uint32_t index = 0;
  __HAL_RCC_CRC_CLK_ENABLE();
	for(index = 0; index < p->init.ChnlNbrIn; index++)
  {
    /* Init PDM filters */
    PDM_FilterHandler[index].bit_order  = PDM_FILTER_BIT_ORDER_LSB;
    PDM_FilterHandler[index].endianness = PDM_FILTER_ENDIANNESS_LE;
    PDM_FilterHandler[index].high_pass_tap = 2104533974;
    PDM_FilterHandler[index].out_ptr_channels = p->init.ChnlNbrOut;
    PDM_FilterHandler[index].in_ptr_channels  = p->init.ChnlNbrIn;
    PDM_Filter_Init((PDM_Filter_Handler_t *)(&PDM_FilterHandler[index]));

    /* PDM lib config phase */
    PDM_FilterConfig[index].output_samples_number = p->init.AudioFreq/1000;
    PDM_FilterConfig[index].mic_gain = 32;//;32;
    PDM_FilterConfig[index].decimation_factor = PDM_FILTER_DEC_FACTOR_128;
    PDM_Filter_setConfig((PDM_Filter_Handler_t *)&PDM_FilterHandler[index], &PDM_FilterConfig[index]);
  }
}

/**
  * @brief  
  * @param  argument: 
  * @retval None
  */
uint8_t AUDIO_IN_I2S2_Init(struct AudioIn * p)
{
	p->init.hAudioIn.Instance 	= I2S2;

	__HAL_I2S_DISABLE(&p->init.hAudioIn);

	p->init.hAudioIn.Init.AudioFreq    = p->init.AudioFreq * 4;
  p->init.hAudioIn.Init.ClockSource  = I2S_CLOCK_PLL;
  p->init.hAudioIn.Init.CPOL         = I2S_CPOL_HIGH;
  p->init.hAudioIn.Init.DataFormat   = I2S_DATAFORMAT_16B;
  p->init.hAudioIn.Init.MCLKOutput   = I2S_MCLKOUTPUT_DISABLE;
  p->init.hAudioIn.Init.Mode         = I2S_MODE_MASTER_RX;
  p->init.hAudioIn.Init.Standard     = I2S_STANDARD_LSB;
	
	  /* Initialize the I2S peripheral with the structure above */  
  if(HAL_I2S_Init(&p->init.hAudioIn) != HAL_OK)
  {
    return AUDIO_ERROR;
  }
  else
  {
    return AUDIO_OK; 
  }
	
}

/**
  * @brief  
  * @param  argument: 
  * @retval None
  */
void AUDIO_IN_MspInit(I2S_HandleTypeDef *hi2s)
{
  static DMA_HandleTypeDef hdma_i2sRx;
  GPIO_InitTypeDef  GPIO_InitStruct;
	
  I2S2_CLK_ENABLE();
  I2S2_SCK_GPIO_CLK_ENABLE();
  I2S2_MOSI_GPIO_CLK_ENABLE();
	
	GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull      = GPIO_NOPULL;
  GPIO_InitStruct.Speed     = GPIO_SPEED_FAST;

  GPIO_InitStruct.Pin       = I2S2_SCK_PIN; 
  GPIO_InitStruct.Alternate = I2S2_SCK_AF;
  HAL_GPIO_Init(I2S2_SCK_GPIO_PORT, &GPIO_InitStruct);
  
  GPIO_InitStruct.Pin       = I2S2_MOSI_PIN ;
  GPIO_InitStruct.Alternate = I2S2_MOSI_AF;
  HAL_GPIO_Init(I2S2_MOSI_GPIO_PORT, &GPIO_InitStruct); 

  /* Enable the DMA clock */
  I2S2_DMAx_CLK_ENABLE();
	
	 if(hi2s->Instance == I2S2)
  {
    /* Configure the hdma_i2sRx handle parameters */   
    hdma_i2sRx.Init.Channel             = I2S2_DMAx_CHANNEL;
    hdma_i2sRx.Init.Direction           = DMA_PERIPH_TO_MEMORY;
    hdma_i2sRx.Init.PeriphInc           = DMA_PINC_DISABLE;
    hdma_i2sRx.Init.MemInc              = DMA_MINC_ENABLE;
    hdma_i2sRx.Init.PeriphDataAlignment = I2S2_DMAx_PERIPH_DATA_SIZE;
    hdma_i2sRx.Init.MemDataAlignment    = I2S2_DMAx_MEM_DATA_SIZE;
    hdma_i2sRx.Init.Mode                = DMA_CIRCULAR;
    hdma_i2sRx.Init.Priority            = DMA_PRIORITY_HIGH;
    hdma_i2sRx.Init.FIFOMode            = DMA_FIFOMODE_DISABLE;         
    hdma_i2sRx.Init.FIFOThreshold       = DMA_FIFO_THRESHOLD_FULL;
    hdma_i2sRx.Init.MemBurst            = DMA_MBURST_SINGLE;
    hdma_i2sRx.Init.PeriphBurst         = DMA_MBURST_SINGLE; 
    
    hdma_i2sRx.Instance = I2S2_DMAx_STREAM;
    
    /* Associate the DMA handle */
    __HAL_LINKDMA(hi2s, hdmarx, hdma_i2sRx);
    
    /* Deinitialize the Stream for new transfer */
    HAL_DMA_DeInit(&hdma_i2sRx);
    
    /* Configure the DMA Stream */
    HAL_DMA_Init(&hdma_i2sRx);      
  }
  
  /* I2S DMA IRQ Channel configuration */
  HAL_NVIC_SetPriority(I2S2_DMAx_IRQ, AUDIO_IN_IRQ_PREPRIO, 5);
  HAL_NVIC_EnableIRQ(I2S2_DMAx_IRQ); 
}

uint16_t testdatam[5]={0x22,0x23,0x7030,0x1040,0x5550};

/**
  * @brief  
  * @param  argument: 
  * @retval None
  */
void AudioInTask(void const * argument)
{
	struct AudioIn * priv = (struct AudioIn *)argument;	
	
	AUDIO_IN_Record(priv,(uint16_t*)&InternalBuffer[0], INTERNAL_BUFF_SIZE);
	uint16_t *ptr;
	
	priv->statistic.queue_error = 0;
	priv->statistic.packet_cnt=0;
	priv->open = false;

	for( ;; )
	{
		if (osSemaphoreWait(priv->s_audio, TIME_WAITING_FOR_INPUT) == osOK)
    {
			  osMutexWait(priv->m_audio, osWaitForever);
				ptr = WrBuffer + AudioBuffOffset;
				// ptr=testdatam;
			  // Send captured 2048 words for FFT processing			
				if(xQueueSend(priv->q_audioToFFT,&ptr, 0) != pdPASS) 
				{
					priv->statistic.queue_error++;
				}

				priv->statistic.packet_cnt++;
			  //HAL_GPIO_TogglePin(GPIOD, LD3_Pin); 

			  osMutexRelease(priv->m_audio);
		}
	}
}

/**
  * @brief  Rx Transfer completed callbacks
  * @param  hi2s: I2S handle
  */
void HAL_I2S_RxCpltCallback(I2S_HandleTypeDef *hi2s)
{
	//HER 1ms bir buraya gireri
	//64 sample PDM verisi 8 sample PCM verisine donusturulur	
	/* PDM to PCM data convert */
  AUDIO_IN_PDMToPCM((uint16_t*)&InternalBuffer[INTERNAL_BUFF_SIZE/2], (uint16_t*)&RecBuf[0]);
  /* Copy PCM data in internal buffer */
  memcpy((uint16_t*)&WrBuffer[ITCounter * (PCM_OUT_SIZE*2)], RecBuf, PCM_OUT_SIZE*4);
	//HAL_GPIO_TogglePin(GPIOD,GPIO_PIN_14);
	if(ITCounter == 127)//320sample tamamlandiysa
  {
		ITCounter++;
	  AudioBuffOffset = 0;
		sAudio.t->open = true;
		osSemaphoreRelease(sAudio.t->s_audio);
  }
	else if (ITCounter == 255)
	{
    AudioBuffOffset = WR_SIZE/2;
		ITCounter = 0;
		sAudio.t->open = true;
		osSemaphoreRelease(sAudio.t->s_audio);
	}
  else
  {
    ITCounter++;
  }
}

/**
  * @brief  Rx Half Transfer completed callbacks.
  * @param  hi2s: I2S handle
  */
void HAL_I2S_RxHalfCpltCallback(I2S_HandleTypeDef *hi2s)
{
	/* PDM to PCM data convert */
  AUDIO_IN_PDMToPCM((uint16_t*)&InternalBuffer[0], (uint16_t*)&RecBuf[0]);
	/* Copy PCM data in internal buffer */
  memcpy((uint16_t*)&WrBuffer[ITCounter * (PCM_OUT_SIZE*2)], RecBuf, PCM_OUT_SIZE*4);
	
	if(ITCounter == 127)//1024 sample tamamlandiysa
  {
    AudioBuffOffset = 0;
		ITCounter = 0;
		sAudio.t->open = true;
		osSemaphoreRelease(sAudio.t->s_audio);
  }
	else if (ITCounter == 255)
	{
    AudioBuffOffset = WR_SIZE/2;
		ITCounter = 0;
		sAudio.t->open = true;
		osSemaphoreRelease(sAudio.t->s_audio);
	}
  else
  {
    ITCounter++;
  }
}
/**
  * @brief  
  * @param  argument: 
  * @retval None
  */
void AUDIO_IN_Record(struct AudioIn * p,uint16_t* pbuf, uint32_t size)
{ 
  /* Start the process receive DMA */
  HAL_I2S_Receive_DMA(&p->init.hAudioIn, pbuf, size);  
}


/**
  * @brief  
  * @param  argument: 
  * @retval None
  */
struct AudioIn * AUDIO_IN_Init()
{
		struct AudioIn * audio_in = (struct AudioIn*)pvPortMalloc(sizeof(struct AudioIn));
			
		audio_in->init.AudioFreq = DEFAULT_AUDIO_IN_FREQ;
		audio_in->init.BitRes = DEFAULT_AUDIO_IN_BIT_RESOLUTION;
		audio_in->init.ChnlNbr = DEFAULT_AUDIO_IN_CHANNEL_NBR;
	
		audio_in->init.ChnlNbrIn = DEFAULT_AUDIO_IN_CHANNEL_NBR;
	 	audio_in->init.ChnlNbrOut = 2;

		AUDIO_IN_ClockConfig();
	  AUDIO_IN_PDMDecoder_Init(audio_in);
		audio_in->init.hAudioIn.Instance = I2S2;
	
		audio_in->ptrTask = AudioInTask;
	
	  osThreadDef(audioInTask, audio_in->ptrTask, osPriorityNormal, 0, 128);
    sAudio.t->audioInTaskHandle = osThreadCreate(osThread(audioInTask), audio_in);
	
		if(HAL_I2S_GetState(&audio_in->init.hAudioIn) == HAL_I2S_STATE_RESET)
		{ 
			AUDIO_IN_MspInit(&audio_in->init.hAudioIn);
		}
		
		AUDIO_IN_I2S2_Init(audio_in);
		
		osSemaphoreDef(SEM);
		audio_in->s_audio = osSemaphoreCreate(osSemaphore(SEM), 1);
		
		osMutexDef(MUTEX);
		audio_in->m_audio= osMutexCreate(osMutex(MUTEX));
		
		audio_in->q_audioToFFT = xQueueCreate(1, sizeof(uint16_t *));
	
		return (struct AudioIn*)audio_in;
}
