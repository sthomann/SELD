/*custom app for fatfs and wav files: 
has read, write and wav file support

still under construction


*/


#ifndef APP_SD_AUDIO_H
#define APP_SD_AUDIO_H

#include <stdint.h>
#include "ff.h"         
#include "ff_gen_drv.h" 
#include "sd_diskio_dma_standalone.h"

// Return codes
#define AUDIO_OK    0
#define AUDIO_ERROR -1

// Function Prototypes
int32_t SD_Audio_Init(void);
int32_t SD_Audio_StartRecording(const char* filename);
int32_t SD_Audio_WriteData(int16_t* pData, uint32_t numSamples);
int32_t SD_Audio_StopRecording(void);
int32_t SD_Audio_OpenForRead(const char* filename);
int32_t SD_Audio_ReadData(uint8_t* pBuffer, uint32_t bytesToRead, uint32_t* bytesRead);
int32_t SD_Audio_ListRootFiles(void);

#endif /* APP_SD_AUDIO_H */