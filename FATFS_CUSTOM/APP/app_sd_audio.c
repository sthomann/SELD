#include "app_sd_audio.h"
#include <string.h>





/* AI GEN STARTING POINT, REMEMBER TO MODIFY WAV ETC LATER

TODO!!!


*/

/* --- Global Variables for FatFs --- */
FATFS SD_FatFs;             /* File system object for SD card */
FIL SD_File;                /* File object */
char SD_Path[4];            /* SD card logical drive path */

/* --- Audio State Variables --- */
uint32_t TotalDataSize = 0; /* Tracks total bytes written to update WAV header later */

/* --- External Driver --- */
/* This is defined in your sd_diskio.c. Make sure the name matches! */
extern const Diskio_drvTypeDef  SD_DMA_Driver;

/* --- WAV Header Constants --- */
/* 22900 Hz, Stereo, 16-bit */
const uint32_t SAMPLE_RATE = 22900;
const uint16_t CHANNELS = 2;
const uint16_t BITS_PER_SAMPLE = 16;

/**
  * @brief  Helper to write a blank WAV header (44 bytes) to reserve space
  */
static FRESULT Write_WAV_Header_Placeholder(void)
{
    uint8_t header[44] = {0}; // Zeroed out buffer
    UINT bytesWritten;
    return f_write(&SD_File, header, 44, &bytesWritten);
}

/**
  * @brief  Goes back to the start of the file and writes the correct file sizes
  */
static FRESULT Update_WAV_Header(void)
{
    uint8_t header[44];
    UINT bytesWritten;
    uint32_t byteRate = SAMPLE_RATE * CHANNELS * (BITS_PER_SAMPLE / 8);
    uint32_t subChunk2Size = TotalDataSize;
    uint32_t chunkSize = 36 + subChunk2Size;

    /* RIFF Chunk */
    memcpy(header, "RIFF", 4);
    memcpy(&header[4], &chunkSize, 4);
    memcpy(&header[8], "WAVE", 4);

    /* fmt Chunk */
    memcpy(&header[12], "fmt ", 4);
    uint32_t subChunk1Size = 16;
    memcpy(&header[16], &subChunk1Size, 4);
    uint16_t audioFormat = 1; // PCM
    memcpy(&header[20], &audioFormat, 2);
    memcpy(&header[22], &CHANNELS, 2);
    memcpy(&header[24], &SAMPLE_RATE, 4);
    memcpy(&header[28], &byteRate, 4);
    uint16_t blockAlign = CHANNELS * (BITS_PER_SAMPLE / 8);
    memcpy(&header[32], &blockAlign, 2);
    memcpy(&header[34], &BITS_PER_SAMPLE, 2);

    /* data Chunk */
    memcpy(&header[36], "data", 4);
    memcpy(&header[40], &subChunk2Size, 4);

    /* Move pointer to beginning of file */
    f_lseek(&SD_File, 0);
    
    /* Overwrite placeholder with real header */
    return f_write(&SD_File, header, 44, &bytesWritten);
}

/**
  * @brief  Initializes SD Card and Mounts File System
  */
int32_t SD_Audio_Init(void)
{
    /* 1. Link the SD Driver */
    /* If your driver is named something else, change &SD_Driver here */
    if (FATFS_LinkDriver(&SD_DMA_Driver, SD_Path) != 0)
    {
        return AUDIO_ERROR;
    }

    /* 2. Mount the SD Card */
    /* Force mount (1) to check if card is inserted immediately */
    if (f_mount(&SD_FatFs, (TCHAR const*)SD_Path, 1) != FR_OK)
    {
        return AUDIO_ERROR;
    }

    return AUDIO_OK;
}

/**
  * @brief  Opens file and prepares for recording
  */
int32_t SD_Audio_StartRecording(const char* filename)
{
    /* Open file with Create Always (overwrites existing) and Write access */
    if (f_open(&SD_File, filename, FA_CREATE_ALWAYS | FA_WRITE) != FR_OK)
    {
        return AUDIO_ERROR;
    }

    /* Reserve 44 bytes for the WAV header */
    if (Write_WAV_Header_Placeholder() != FR_OK)
    {
        return AUDIO_ERROR;
    }

    TotalDataSize = 0;
    return AUDIO_OK;
}

/**
  * @brief  Writes a chunk of audio data to the SD card
  * @param  pData: Pointer to your 16-bit PCM buffer (PCM_OUT)
  * @param  numSamples: Number of 16-bit SAMPLES (not bytes)
  */
int32_t SD_Audio_WriteData(int16_t* pData, uint32_t numSamples)
{
    UINT bytesWritten;
    /* Calculate bytes: 16-bit samples * 2 bytes/sample */
    uint32_t bytesToWrite = numSamples * 2; 

    /* Write data to file */
    if (f_write(&SD_File, (uint8_t*)pData, bytesToWrite, &bytesWritten) != FR_OK)
    {
        return AUDIO_ERROR;
    }

    /* Verify all data was written */
    if (bytesWritten != bytesToWrite)
    {
        return AUDIO_ERROR; /* Disk full or error */
    }

    TotalDataSize += bytesWritten;
    return AUDIO_OK;
}

/**
  * @brief  Finalizes the file (Updates Header, Closes File)
  */
int32_t SD_Audio_StopRecording(void)
{
    /* 1. Go back and write the correct WAV header size */
    if (Update_WAV_Header() != FR_OK)
    {
        f_close(&SD_File);
        return AUDIO_ERROR;
    }

    /* 2. Close the file to flush data */
    if (f_close(&SD_File) != FR_OK)
    {
        return AUDIO_ERROR;
    }

    /* Optional: Unmount if you are done completely */
    // f_mount(NULL, (TCHAR const*)SD_Path, 0); 

    return AUDIO_OK;
}

/**
  * @brief  Opens an existing file for read access
  * @param  filename: Name of the file (e.g., "AUDIO.WAV")
  */
int32_t SD_Audio_OpenForRead(const char* filename)
{
    /* Open file with Read access */
    if (f_open(&SD_File, filename, FA_READ) != FR_OK)
    {
        return AUDIO_ERROR;
    }
    // Optionally read the WAV header here if you need file information
    return AUDIO_OK;
}

/**
  * @brief  Reads a chunk of data from the currently opened file
  * @param  pBuffer: Pointer to the buffer where data will be stored
  * @param  bytesToRead: Number of bytes requested
  * @param  bytesRead: Pointer to a variable to store the actual number of bytes read
  */
int32_t SD_Audio_ReadData(uint8_t* pBuffer, uint32_t bytesToRead, uint32_t* bytesRead)
{
    FRESULT res;

    res = f_read(&SD_File, pBuffer, bytesToRead, (UINT*)bytesRead);

    if (res != FR_OK)
    {
        return AUDIO_ERROR;
    }

    /* Check if we reached the end of the file (EOF) */
    if (*bytesRead == 0)
    {
        return 1; // Return a custom code (1) for EOF
    }

    return AUDIO_OK;
}
int32_t SD_Audio_ListRootFiles(void)
{
    FRESULT fr;
    DIR dj;           // Directory object
    FILINFO fno;      // File information structure
    char path[] = "0:/"; // Root directory path

    printf("--- Listing Files in %s ---\n", path);

    // 1. Open the directory
    fr = f_opendir(&dj, path);
    if (fr != FR_OK)
    {
        printf("ERROR: f_opendir failed (%d)\n", fr);
        return AUDIO_ERROR;
    }

    // 2. Loop through all entries
    while (1)
    {
        fr = f_readdir(&dj, &fno); // Read a directory item

        if (fr != FR_OK || fno.fname[0] == 0) 
        {
            // Break if error occurred or end of directory reached (fname[0] == 0)
            break; 
        }

        // Skip current ('.') and parent ('..') directory entries
        if (fno.fname[0] == '.') 
        {
            continue;
        }

        // 3. Print the entry information
        if (fno.fattrib & AM_DIR)
        {
            // Entry is a Directory
            printf("[DIR] %s\n", fno.fname);
        }
        else
        {
            // Entry is a File
            // Note: fno.fsize is file size in bytes
            printf("[FIL] %s (%lu bytes)\n", fno.fname, fno.fsize);
        }
    }

    // 4. Close the directory
    f_closedir(&dj);
    printf("--- End of List ---\n");

    return AUDIO_OK;
}