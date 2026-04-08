#include "main.h"
#include "fatfs.h"
#include <string.h>
#include <stdio.h>

#define SAMPLE_RATE        44100u   /* Hz  */
#define BITS_PER_SAMPLE    16u      /* bits */
#define NUM_CHANNELS       1u       /* mono */
#define RECORD_MAX_SECONDS 60u      

#define DMA_HALF_LEN       1024u
#define DMA_FULL_LEN       (DMA_HALF_LEN * 2u)

typedef struct __attribute__((packed)) {

    char     riff_id[4];    
    uint32_t riff_size;      
    char     wave_id[4];    
    char     fmt_id[4];       
    uint32_t fmt_size;       
    uint16_t audio_format;   
    uint16_t num_channels;
    uint32_t sample_rate;
    uint32_t byte_rate;      
    uint16_t block_align;     
    uint16_t bits_per_sample;

    char     data_id[4];    
    uint32_t data_size;      
} WAV_Header_t;

extern ADC_HandleTypeDef  hadc1;
extern TIM_HandleTypeDef  htim2;
extern SPI_HandleTypeDef  hspi1;

static volatile uint16_t  adc_dma_buf[DMA_FULL_LEN];

static int16_t            pcm_buf[DMA_HALF_LEN];

static FATFS    fs;
static FIL      wav_file;
static uint32_t bytes_written_total = 0u;

typedef enum {
    STATE_IDLE      = 0,
    STATE_RECORDING = 1,
    STATE_STOPPING  = 2
} RecorderState_t;

static volatile RecorderState_t rec_state      = STATE_IDLE;
static volatile uint8_t         dma_half_ready = 0u;
static volatile uint8_t         dma_full_ready = 0u;
static volatile uint8_t         btn_pressed    = 0u;

#define LED_RED_ON()    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET)
#define LED_RED_OFF()   HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET)
#define LED_GREEN_ON()  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_SET)
#define LED_GREEN_OFF() HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_RESET)

static void wav_fill_header(WAV_Header_t *hdr, uint32_t data_bytes)
{
    memcpy(hdr->riff_id,  "RIFF", 4);
    hdr->riff_size       = 36u + data_bytes;
    memcpy(hdr->wave_id,  "WAVE", 4);
    memcpy(hdr->fmt_id,   "fmt ", 4);
    hdr->fmt_size        = 16u;
    hdr->audio_format    = 1u;  
    hdr->num_channels    = NUM_CHANNELS;
    hdr->sample_rate     = SAMPLE_RATE;
    hdr->byte_rate       = SAMPLE_RATE * NUM_CHANNELS * (BITS_PER_SAMPLE / 8u);
    hdr->block_align     = NUM_CHANNELS * (BITS_PER_SAMPLE / 8u);
    hdr->bits_per_sample = BITS_PER_SAMPLE;
    memcpy(hdr->data_id,  "data", 4);
    hdr->data_size       = data_bytes;
}

static void wav_patch_header(FIL *fp, uint32_t data_bytes)
{
    WAV_Header_t hdr;
    UINT bw;
    wav_fill_header(&hdr, data_bytes);
    f_lseek(fp, 0);
    f_write(fp, &hdr, sizeof(hdr), &bw);
}


static void convert_adc_to_pcm(const volatile uint16_t *src, int16_t *dst, uint16_t len)
{
    for (uint16_t i = 0u; i < len; i++) {
        int32_t s = (int32_t)src[i] - 2048; 
        s = s * 16;                          
        if (s >  32767) s =  32767;
        if (s < -32768) s = -32768;
        dst[i] = (int16_t)s;
    }
}

static FRESULT open_wav_file(void)
{
    char fname[32];
    WAV_Header_t hdr;
    UINT bw;

    static uint16_t rec_index = 0u;
    rec_index++;
    snprintf(fname, sizeof(fname), "REC_%03u.WAV", rec_index);

    FRESULT res = f_open(&wav_file, fname, FA_CREATE_ALWAYS | FA_WRITE);
    if (res != FR_OK) return res;

    wav_fill_header(&hdr, 0u);
    res = f_write(&wav_file, &hdr, sizeof(hdr), &bw);
    return res;
}

static void close_wav_file(void)
{
    wav_patch_header(&wav_file, bytes_written_total);
    f_close(&wav_file);
}

static void recording_start(void)
{
    bytes_written_total = 0u;
    dma_half_ready      = 0u;
    dma_full_ready      = 0u;

    if (open_wav_file() != FR_OK) {
        f_mount(&fs, "", 1);
        if (open_wav_file() != FR_OK) return;
    }

    rec_state = STATE_RECORDING;
    LED_RED_ON();
    LED_GREEN_OFF();

    HAL_ADC_Start_DMA(&hadc1, (uint32_t *)adc_dma_buf, DMA_FULL_LEN);
    HAL_TIM_Base_Start(&htim2);
}

static void recording_stop(void)
{
    rec_state = STATE_STOPPING;

    HAL_TIM_Base_Stop(&htim2);
    HAL_ADC_Stop_DMA(&hadc1);

    close_wav_file();

    rec_state = STATE_IDLE;
    LED_RED_OFF();
    LED_GREEN_ON();
}


static void write_pcm_chunk(const volatile uint16_t *src)
{
    UINT bw;
    convert_adc_to_pcm(src, pcm_buf, DMA_HALF_LEN);
    f_write(&wav_file, pcm_buf, DMA_HALF_LEN * sizeof(int16_t), &bw);
    bytes_written_total += bw;

    uint32_t max_bytes = SAMPLE_RATE * NUM_CHANNELS * (BITS_PER_SAMPLE / 8u)
                         * RECORD_MAX_SECONDS;
    if (bytes_written_total >= max_bytes) {
        btn_pressed = 1u; 
    }
}

void HAL_ADC_ConvHalfCpltCallback(ADC_HandleTypeDef *hadc)
{
    if (hadc->Instance == ADC1 && rec_state == STATE_RECORDING) {
        dma_half_ready = 1u;
    }
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
{
    if (hadc->Instance == ADC1 && rec_state == STATE_RECORDING) {
        dma_full_ready = 1u;
    }
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if (GPIO_Pin == GPIO_PIN_13) {
        static uint32_t last_tick = 0u;
        uint32_t now = HAL_GetTick();
        if ((now - last_tick) > 200u) {
            btn_pressed  = 1u;
            last_tick    = now;
        }
    }
}

void Recorder_Init(void)
{
    f_mount(&fs, "", 1);

    LED_GREEN_ON();
    LED_RED_OFF();
}

void Recorder_Run(void)
{
    if (btn_pressed) {
        btn_pressed = 0u;
        if (rec_state == STATE_IDLE) {
            recording_start();
        } else if (rec_state == STATE_RECORDING) {
            recording_stop();
        }
    }

    if (dma_half_ready && rec_state == STATE_RECORDING) {
        dma_half_ready = 0u;
        write_pcm_chunk(&adc_dma_buf[0]);         
    }

    if (dma_full_ready && rec_state == STATE_RECORDING) {
        dma_full_ready = 0u;
        write_pcm_chunk(&adc_dma_buf[DMA_HALF_LEN]);
    }
}



   int main(void) {
       HAL_Init();
       SystemClock_Config();
       MX_GPIO_Init();
       MX_DMA_Init();
       MX_ADC1_Init();
       MX_TIM2_Init();
       MX_SPI1_Init();
       MX_FATFS_Init();

       Recorder_Init();         
       while (1) {
           Recorder_Run();      
       }
   }

