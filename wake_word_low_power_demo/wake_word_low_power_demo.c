/*
 * Copyright (c) 2015, Freescale Semiconductor, Inc.
 * Copyright 2020 NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

/* FreeRTOS kernel includes. */
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "timers.h"
#include "semphr.h"

/* Freescale includes. */
#include "fsl_device_registers.h"
#include "fsl_debug_console.h"
#include "board.h"

#include "pin_mux.h"
#include "clock_config.h"

#include "fsl_rdc.h"
#include "fsl_gpio.h"
#include "fsl_gpc.h"
#include "fsl_gpt.h"
#include "fsl_iomuxc.h"
#include "fsl_mu.h"

#include "fsl_sai.h"
#include "fsl_codec_common.h"
#include "fsl_wm8524.h"
#include "fsl_common.h"
#include "fsl_codec_adapter.h"

#include "public/rdsp_voicespot.h"
#include "public/rdsp_voicespot_utils.h"
#include "models/Alexa/EN-US/Alexa_model-standard_en-US_4_SmallVocabularyCompatible.h"
#include "models/Alexa/EN-US/Alexa_model-standard_en-US_4_SmallVocabularyCompatible_params.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/

#define DEMO_SAI (I2S3)
#define DEMO_SAI_5 (I2S5)
#define DEMO_SAI_CLK_FREQ                                                                  \
    (CLOCK_GetPllFreq(kCLOCK_AudioPll1Ctrl) / (CLOCK_GetRootPreDivider(kCLOCK_RootSai5)) / \
     (CLOCK_GetRootPostDivider(kCLOCK_RootSai5)))
#define DEMO_CODEC_WM8524 (1)
#define DEMO_CODEC_BUS_PIN (NULL)
#define DEMO_CODEC_BUS_PIN_NUM (0)
#define DEMO_CODEC_MUTE_PIN (GPIO5)
#define DEMO_CODEC_MUTE_PIN_NUM (21)
/*set Bclk source to Mclk clock*/
#define DEMO_SAI_CLOCK_SOURCE (1U)

#define DEMO_AUDIO_SAMPLE_RATE (kSAI_SampleRate16KHz)
#define DEMO_AUDIO_MASTER_CLOCK DEMO_SAI_CLK_FREQ
/* demo audio data channel */
#define DEMO_AUDIO_DATA_CHANNEL (2U)
/* demo audio bit width */
#define DEMO_AUDIO_BIT_WIDTH kSAI_WordWidth32bits

#define RECORDING_WIN (2)
#define AVERAGING_WINDOW_LEN (3)

#define BUFFER_SIZE (256U)

#define BUFFER_NUMBER (4)

#define APP_TASK_STACK_SIZE (256)

#define APP_PowerUpSlot (5U)
#define APP_PowerDnSlot (6U)

#define RDC_DISABLE_A53_ACCESS 0xFC
#define RDC_DISABLE_M4_ACCESS 0xF3

/* Task priorities. */
#define hello_task_PRIORITY      (configMAX_PRIORITIES - 1)

#define ServiceFlagAddr SRC->GPR9
#define ServiceBusy (0x5555U)
#define ServiceIdle (0x0U)

/*******************************************************************************
 * Prototypes
 ******************************************************************************/

void BOARD_WM8524_Mute_GPIO(uint32_t output);
void BOARD_MASTER_CLOCK_CONFIG(void);

rdsp_voicespot_control* voicespot_control=NULL; 		// Pointer to VoiceSpot control structure
int32_t voicespot_handle=0;	
int32_t processing_level;
int32_t test_cnt = 0 ;

int32_t num_scores=0;
int32_t num_samples_per_frame;
int32_t voicespot_status;						// Return status for VoiceSpot - 0 indicates success (see .h for return codes)
int32_t** sfb_output = NULL;
int32_t* frame_buffer = NULL; // Frame buffer into VoiceSpot
int32_t* scores = NULL;

int32_t frame_count=0;
int32_t num_frames_per_second;
int32_t num_outputs=1;
void run_VOICESPOT();
char** voicespot_class_string;
int32_t buffer_size_num_frames = 40;
int32_t init_phase = 0;

int32_t last_event = 0;
int32_t bufferLength = 0;
int32_t readIndex = 0;
int32_t writeIndex = 0;
int32_t writeIndex_Alexa = 0;
int32_t number_samples=200;
int32_t condition;

/*******************************************************************************
 * Variables
 ******************************************************************************/
static wm8524_config_t wm8524Config = {
    .setMute     = BOARD_WM8524_Mute_GPIO,
    .setProtocol = NULL,
    .protocol    = kWM8524_ProtocolI2S,
};
codec_config_t boardCodecConfig = {.codecDevType = kCODEC_WM8524, .codecDevConfig = &wm8524Config};

static volatile bool isFinished = false;
extern codec_config_t boardCodecConfig;
#if (defined(FSL_FEATURE_SAI_HAS_MCR) && (FSL_FEATURE_SAI_HAS_MCR)) || \
    (defined(FSL_FEATURE_SAI_HAS_MCLKDIV_REGISTER) && (FSL_FEATURE_SAI_HAS_MCLKDIV_REGISTER))
sai_master_clock_t mclkConfig = {
#if defined(FSL_FEATURE_SAI_HAS_MCR) && (FSL_FEATURE_SAI_HAS_MCR)
    .mclkOutputEnable = true,
#if !(defined(FSL_FEATURE_SAI_HAS_NO_MCR_MICS) && (FSL_FEATURE_SAI_HAS_NO_MCR_MICS))
    .mclkSource = kSAI_MclkSourceSysclk,
#endif
#endif
};
#endif

codec_handle_t codecHandle;

AT_NONCACHEABLE_SECTION_ALIGN(static uint8_t Buffer[BUFFER_SIZE * BUFFER_NUMBER], 4);
AT_NONCACHEABLE_SECTION_ALIGN(static int32_t circular_buffer[1600], 4);

sai_handle_t txHandle = {0}, rxHandle = {0};
static uint32_t tx_index = 0U, rx_index = 0U;


/*******************************************************************************
 * Code
 ******************************************************************************/
void BOARD_WM8524_Mute_GPIO(uint32_t output)
{
    GPIO_PinWrite(DEMO_CODEC_MUTE_PIN, DEMO_CODEC_MUTE_PIN_NUM, output);
}

void BOARD_MASTER_CLOCK_CONFIG(void)
{
    mclkConfig.mclkOutputEnable = true, mclkConfig.mclkHz = DEMO_AUDIO_MASTER_CLOCK;
    mclkConfig.mclkSourceClkHz = DEMO_SAI_CLK_FREQ;
    SAI_SetMasterClockConfig(DEMO_SAI, &mclkConfig);
    SAI_SetMasterClockConfig(DEMO_SAI_5, &mclkConfig);
}

void Peripheral_RdcSetting(void)
{
    rdc_periph_access_config_t periphConfig;
    
    RDC_GetDefaultPeriphAccessConfig(&periphConfig);

    /* Do not allow the A53 domain(domain0) to access the following peripherals. */
    periphConfig.policy = RDC_DISABLE_A53_ACCESS;
    
    periphConfig.periph = kRDC_Periph_UART4;
    RDC_SetPeriphAccessConfig(RDC, &periphConfig);
   
    periphConfig.periph = kRDC_Periph_SAI3_ACCESS;
    RDC_SetPeriphAccessConfig(RDC, &periphConfig);

    periphConfig.periph = kRDC_Periph_SAI5_ACCESS;
    RDC_SetPeriphAccessConfig(RDC, &periphConfig);

    periphConfig.periph = kRDC_Periph_I2C3;
    RDC_SetPeriphAccessConfig(RDC, &periphConfig);
    
    periphConfig.periph = kRDC_Periph_I2C4;
    RDC_SetPeriphAccessConfig(RDC, &periphConfig);
    
    periphConfig.periph = kRDC_Periph_GPIO5;
    RDC_SetPeriphAccessConfig(RDC, &periphConfig);
}



static void rx_callback(I2S_Type *base, sai_handle_t *handle, status_t status, void *userData)
{
    sai_transfer_t xfer = { 0 };
    if(kStatus_SAI_RxError == status)
    {
      SAI_RxClearStatusFlags(DEMO_SAI, kSAI_FIFOErrorFlag);
      SAI_RxSoftwareReset(DEMO_SAI, kSAI_ResetTypeFIFO);
      SAI_TransferAbortReceive(DEMO_SAI, &rxHandle);

      xfer.data = Buffer + rx_index * BUFFER_SIZE;
      xfer.dataSize = BUFFER_SIZE;
      SAI_TransferReceiveNonBlocking(DEMO_SAI, &rxHandle, &xfer);
    }
    else
    {
      rx_index = (rx_index + 1) % BUFFER_NUMBER;        
      xfer.data = Buffer + rx_index * BUFFER_SIZE;
      xfer.dataSize = BUFFER_SIZE;
      SAI_TransferReceiveNonBlocking(DEMO_SAI, &rxHandle, &xfer);  
    }
}

static void tx_callback(I2S_Type *base, sai_handle_t *handle, status_t status, void *userData)
{
    sai_transfer_t xfer = { 0 };

    if(kStatus_SAI_TxError == status)
    {
      SAI_TxClearStatusFlags(DEMO_SAI, kSAI_FIFOErrorFlag);
      SAI_TxSoftwareReset(DEMO_SAI, kSAI_ResetTypeFIFO);
      SAI_TransferAbortSend(DEMO_SAI, &txHandle);

      xfer.data = Buffer + tx_index * BUFFER_SIZE;
      xfer.dataSize = BUFFER_SIZE;
      SAI_TransferSendNonBlocking(DEMO_SAI, &txHandle, &xfer);
    }
    else
    { 
        if(init_phase==0){
            frame_buffer = (int32_t*) rdsp_malloc(num_samples_per_frame * sizeof(int32_t)); // Frame buffer into VoiceSpot
            scores = (int32_t*) rdsp_malloc( sizeof(int32_t));
            init_phase=1;
        }

        memmove(circular_buffer+writeIndex*(BUFFER_SIZE/4),Buffer + tx_index * BUFFER_SIZE,256);
        writeIndex = writeIndex+1;
        bufferLength=bufferLength+64;

        if (bufferLength >= number_samples) 
        {
            memmove(frame_buffer,circular_buffer+readIndex*200,800);
            for (int32_t i = 0; i < 200; i++) {
                ((float*)frame_buffer)[i] = ((int32_t*)frame_buffer)[i] * (1.0f / 2147483648.0f); 
            }
            processing_level = RDSP_PROCESSING_LEVEL__FULL;

            //Run VoiceSpot Process on the audio
            voicespot_status = rdspVoiceSpot_Process(voicespot_control, voicespot_handle, processing_level, (uint8_t*) frame_buffer, &num_scores, scores, (uint8_t**) sfb_output);
            if (voicespot_status != RDSP_VOICESPOT_OK)
                PRINTF("Warning: VoiceSpot returned error code %d\n", (int) voicespot_status);

            //Analyse the VoiceSpot Results to check if the Keyword is detected 
            run_VOICESPOT(); 
            readIndex = readIndex+1;            
            bufferLength=bufferLength-200;
            if (readIndex == 8)
            {
                readIndex = 0;
            }  
        }   

        if (writeIndex == 25)
        {
            writeIndex = 0;
        }        
        
        tx_index = (tx_index + 1) % BUFFER_NUMBER;  
        xfer.data = Buffer + tx_index * BUFFER_SIZE;
        xfer.dataSize = BUFFER_SIZE;
        SAI_TransferSendNonBlocking(DEMO_SAI, &txHandle, &xfer);
    }
}

void app_task(void *param)
{
    ServiceFlagAddr = ServiceBusy; 
    scores = (int32_t*) rdsp_malloc( sizeof(int32_t));
    sai_transfer_t xfer;
    xfer.data = Buffer + rx_index * BUFFER_SIZE;
    xfer.dataSize = BUFFER_SIZE;
    SAI_TransferReceiveNonBlocking(DEMO_SAI_5, &rxHandle, &xfer);

    xfer.data = Buffer + tx_index * BUFFER_SIZE;
    xfer.dataSize = BUFFER_SIZE;
    SAI_TransferSendNonBlocking(DEMO_SAI, &txHandle, &xfer);
    
    PRINTF("Say Alexa to wake up the Cortex A\r\n");
    while (1)
     {
    }

}


/*!
 * @brief Main function
 */
int main(void)
{
    sai_transceiver_t config;
    sai_transceiver_t config_SAI5;
    int32_t i = 0;
    /* Init board hardware. */
    /* Board specific RDC settings */
    BOARD_RdcInit();
    Peripheral_RdcSetting();
    BOARD_InitPins();
    BOARD_BootClockRUN();
    BOARD_InitDebugConsole();
    BOARD_InitMemory();
    MU_Init(MUB);

    /*
     * In order to wakeup M4 from LPM, all PLLCTRLs need to be set to "NeededRun"
     */
    for (i = 0; i < 39; i++)
    {
        CCM->PLL_CTRL[i].PLL_CTRL = kCLOCK_ClockNeededRun;
    }
  
    CLOCK_SetRootMux(kCLOCK_RootSai3, kCLOCK_SaiRootmuxAudioPll1); /* Set SAI source to AUDIO PLL1 786432000HZ*/
    CLOCK_SetRootDivider(kCLOCK_RootSai3, 1U, 32U);                /* Set root clock to 786432000HZ / 32 = 24.576M */

    CLOCK_SetRootMux(kCLOCK_RootSai5, kCLOCK_SaiRootmuxAudioPll1); /* Set SAI source to AUDIO PLL1 786432000HZ*/
    CLOCK_SetRootDivider(kCLOCK_RootSai5, 1U, 32U);                /* Set root clock to 786432000HZ / 32 = 24.576M */
 
 
    /* gpio initialization */
    gpio_pin_config_t gpioConfig = {kGPIO_DigitalOutput, 1};
    GPIO_PinInit(DEMO_CODEC_MUTE_PIN, DEMO_CODEC_MUTE_PIN_NUM, &gpioConfig);

    /* SAI init */
    SAI_Init(DEMO_SAI);
    SAI_Init(DEMO_SAI_5);
    SAI_TransferTxCreateHandle(DEMO_SAI, &txHandle, tx_callback, NULL);
    //SAI_TransferTxCreateHandle(DEMO_SAI_5, &txHandle, tx_callback, NULL);
    SAI_TransferRxCreateHandle(DEMO_SAI_5, &rxHandle, rx_callback, NULL);
    /* I2S mode configurations */
    SAI_GetClassicI2SConfig(&config, DEMO_AUDIO_BIT_WIDTH, kSAI_MonoLeft, kSAI_Channel0Mask);
    SAI_GetClassicI2SConfig(&config_SAI5, DEMO_AUDIO_BIT_WIDTH, kSAI_MonoLeft, kSAI_Channel0Mask);
    SAI_TransferTxSetConfig(DEMO_SAI, &txHandle, &config);
    config.masterSlave = kSAI_Master;
    config.syncMode = kSAI_ModeAsync;
    config_SAI5.masterSlave = kSAI_Master;
    config_SAI5.syncMode = kSAI_ModeAsync;


    SAI_TransferRxSetConfig(DEMO_SAI_5, &rxHandle, &config_SAI5);

    /* set bit clock divider */
    SAI_TxSetBitClockRate(DEMO_SAI, DEMO_AUDIO_MASTER_CLOCK, DEMO_AUDIO_SAMPLE_RATE, DEMO_AUDIO_BIT_WIDTH,
                          DEMO_AUDIO_DATA_CHANNEL);

    SAI_RxSetBitClockRate(DEMO_SAI_5, DEMO_AUDIO_MASTER_CLOCK, DEMO_AUDIO_SAMPLE_RATE, DEMO_AUDIO_BIT_WIDTH,
                          DEMO_AUDIO_DATA_CHANNEL);

    /* master clock configurations */
    BOARD_MASTER_CLOCK_CONFIG();

    /* Use default setting to init codec */
    CODEC_Init(&codecHandle, &boardCodecConfig);

    PRINTF("\r\n#################### WAKE WORD LOW POWER DEMO ####################\n\r\n");
    PRINTF("    Build Time: %s--%s \r\n", __DATE__, __TIME__);


    /************* RETUNE INIT *************/
    int32_t data_type = RDSP_DATA_TYPE__FLOAT32; 	// Floating point input, floating-point mode computations

    voicespot_status = rdspVoiceSpot_CreateControl(&voicespot_control, data_type); 				// Create VoiceSpot control structure
    PRINTF("rdspVoiceSpot_CreateControl: voicespot_status = %d\n\r", (int) voicespot_status);
																// Declare voiceSpot handle
    int32_t enable_highpass_filter =0;												// Enable VoiceSpot high-pass filter
    int32_t generate_output = 0;												// Do not generate any output from the processes
    voicespot_status = rdspVoiceSpot_CreateInstance(voicespot_control, &voicespot_handle, enable_highpass_filter, generate_output); // Create VoiceSpot instance
    PRINTF("rdspVoiceSpot_CreateInstance: voicespot_status = %d\n\r", (int) voicespot_status);
													// Do not generate any output from the processes

    // Adaptive threshold modes
	// 0: fixed threshold (rdsp_event_threshold)
	// 1: adaptive threshold
	// 2: adaptive sensitivity
	// 3: adaptive threshold + adaptive sensitivity
	int32_t adapt_threshold_mode = 3;
	voicespot_status = rdspVoiceSpot_EnableAdaptiveThreshold(voicespot_control, voicespot_handle, adapt_threshold_mode); // Create VoiceSpot VAD
	PRINTF("rdspVoiceSpot_EnableAdaptiveThreshold: voicespot_status = %d\n\r", (int) voicespot_status);

    uint8_t *model_blob = NULL;										// Pointer to voice model
    uint32_t model_blob_size = 0;									// Size of voice model

    // Get model_blob from header file
    model_blob_size = sizeof(model_blob_header);
    model_blob = rdsp_malloc_align(model_blob_size, 16); // Allocate memory for an aligned copy of the model contained in header file. This allows dynamic modification for e.g. weight permutation depending on target platform
    rdsp_memcpy(model_blob, (uint8_t*) model_blob_header, model_blob_size); // Copy model_blob from header file


	voicespot_status = rdspVoiceSpot_OpenInstance(voicespot_control, voicespot_handle, model_blob_size, model_blob, 0); // Open the VoiceSpot instance

    PRINTF("rdspVoiceSpot_OpenInstance: voicespot_status = %d\n\r", (int) voicespot_status);

    // Set up parameters using a parameter blob
	uint8_t *param_blob = NULL;
    
	param_blob = (uint8_t*) param_blob_header;
	voicespot_status = rdspVoiceSpot_SetParametersFromBlob(voicespot_control, voicespot_handle, param_blob);
	PRINTF("rdspVoiceSpot_SetParametersFromBlob: voicespot_status = %d\n\r", (int) voicespot_status);
  
    rdsp_voicespot_version voicespot_version;
    char* voicespot_model_string;
    rdspVoiceSpot_GetLibVersion(voicespot_control, &voicespot_version);
    PRINTF("VoiceSpot library version: %d.%d.%d.%u\n\r", (int) voicespot_version.major, (int) voicespot_version.minor, (int) voicespot_version.patch, (unsigned int) voicespot_version.build);
    rdspVoiceSpot_GetModelInfo(voicespot_control, voicespot_handle, &voicespot_version, &voicespot_model_string, &voicespot_class_string, &num_samples_per_frame, &num_outputs);
    PRINTF("VoiceSpot model version: %d.%d.%d\n\r", (int) voicespot_version.major, (int) voicespot_version.minor, (int) voicespot_version.patch);
    PRINTF("VoiceSpot model string: %s\n\r", voicespot_model_string);
    num_frames_per_second = 16000 / num_samples_per_frame;
    // Adaptive power state modes
    // 0: Off/manual
    // 1: Auto
    
    int32_t power_state_buffer_length_num_frames_in = -1;
    if (power_state_buffer_length_num_frames_in >= 0)
        buffer_size_num_frames = power_state_buffer_length_num_frames_in;

    if (generate_output)
        sfb_output = (int32_t**) rdsp_malloc(num_samples_per_frame * sizeof(uint32_t));

    /************* RETUNE INIT END *************/

    if (xTaskCreate(app_task, "APP_TASK", APP_TASK_STACK_SIZE, NULL, hello_task_PRIORITY + 1, NULL) != pdPASS)
    {
        PRINTF("\r\nFailed to create application task\r\n");
        while (1)
            ;
    }

    vTaskStartScheduler();
    for (;;)
        ;
}


/*!
 * @brief Task responsible for printing of "Hello world." message.
 */

void run_VOICESPOT()
{ 
    // event_thresholds is the array of manually set minimum thresholds for trigger event per class. NULL means automatic, i.e. no manually set minimum thresholds.
    int32_t* event_thresholds = NULL;   // event_thresholds = NULL means automatic, i.e. no manually set minimum thresholds.
    int32_t processing_period = 4;
    if (num_scores > 0) {
        int32_t score_index = rdspVoiceSpot_CheckIfTriggered(voicespot_control, voicespot_handle, scores, condition, event_thresholds, processing_period);      
        if (score_index >= 0) {
            // We found a trigger, so estimate the starting point
            int32_t start_offset_samples = 0;
            int32_t timing_accuracy = 4; // Accuracy of the timing estimate, in frames
            voicespot_status = rdspVoiceSpot_EstimateStart(voicespot_control, voicespot_handle, score_index, -1, timing_accuracy, &start_offset_samples); // Comment out this line if timing estimation is not to be used

            if (voicespot_status != RDSP_VOICESPOT_OK)
                PRINTF("Warning: VoiceSpot returned error code %d\n\r", (int)voicespot_status);
            PRINTF("Trigger event found: class_string = %s, Score = %d\n\r", voicespot_class_string[score_index], (int) scores[score_index]);
            MU_SendMsg(MUB, 1, 2); //wake up Cortex A
            PRINTF("set mu interrupt MU_SendMsg\r\n"); 
            frame_count = 0;
        }
    }
   
    if(frame_count>=160)
    {
        condition = 1;
    }
    else{
        condition = 0;
        frame_count ++;

    }
}
