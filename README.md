# stm32F4_dsp_microphone_fft_rtos
stm32F4 Discovery FFT RTOS

Bu çalışmada stm32f4 discovery kitinin üzerindeki mp45dt02 mikrofondan 8khz örnekleme frekansı ile örmnekler 
DMA yardımıyla buffera yazılıyor. Daha sonra her 2048 sample RTOS yardımıyla proccesing işlemi için queue ile aktrılıyor ve FFT si alınıyor.
Andorid telefone sinyal generater diye bir program kurularak Nyqusit teoremi nedeniyle en fazla 4000 khz sinyali hesaplayabiliyor. Telefondan
400Hz sinyal oluşturularak stm32 sürekli dinliyor ve hesaplama yaparak tespit edilen frekansı float32_t res; değişkenine yazıyor. 


HAKAN AYDIN
aydinhakan91@gmail.com
