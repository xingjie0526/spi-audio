/*
 * Copyright (C) 2016 AISpeech, Inc.
 *
 * Author: caijun.yang <caijun.yang@aispeech.com>
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <signal.h>
#include <string.h>
#include <pthread.h>
#include "spi_audio.h"

int capturing = 1;

unsigned int capture_sample(FILE *file, struct spi_audio *audio);

static pthread_t gSpiWriteThread ;
static struct spi_audio *g_pcm = NULL;

const unsigned char dummy_frame[] = {
    '0', 'a', 'b', 'c', 'd', 'e', 'f', 'g', 'h', 'i', 'j', 'k', 'l', 'm', 'n', 'o',
};

void sigint_handler(int sig) {
    capturing = 0;
}

static void *spi_write_dummy_data_run(void *arg) {
    unsigned char *buffer;
    int frame_size = sizeof(dummy_frame);
    int frames = 800 * 2;
    int i;

    printf("spi_write_dummy_data_run()\n");
    buffer = malloc(frame_size * frames);
    if (!buffer) {
        fprintf(stderr, "Unable to allocate %d bytes\n", frame_size * frames);
        return NULL;
    }

    for(i=0; (i + frame_size) < (frame_size * frames); i=i+frame_size) {
        memcpy(&buffer[i], dummy_frame, frame_size);
        buffer[i] = i%5;
    }

    while (capturing && spi_audio_is_ready(g_pcm) && spi_audio_write(g_pcm, buffer, frame_size * frames) > 0) {
        printf("spi_audio_write() frame_size=%d \n", frame_size);
    }

    printf("spi_write_dummy_data_run() end\n");

    return NULL;
}

static struct spi_audio_config sDefaultSpiAudioConfig = {
    .channels = 8,
    .rate = 16000,
    .periodSize = 320, /* (sample_rate * BUFF_DURATION_MS) / 1000 = 16000 * 20 / 1000 = 256 */
    .periodCount = 10,
    .format = SPI_AUDIO_FORMAT_S16_LE,
};

int main(int argc, char **argv)
{
    FILE *file;
    unsigned int bytes;
    unsigned char  testFlag = 0xFF;
    int ret = -1;

    if (argc == 2) {
        testFlag = (unsigned char) argv[1][0];
    }
    printf("main() testFlag=%d \n", testFlag);
    printf("main() T=%d \n", 'T');

Loop:
    g_pcm = spi_audio_open(&sDefaultSpiAudioConfig);
    if (!g_pcm || !spi_audio_is_ready(g_pcm)) {
        fprintf(stderr, "Unable to open SPI PCM device\n");
        return -1;
    }
    if (testFlag != 0xFF) {
        spi_audio_set_em_mode(g_pcm, testFlag);
    }
    
    bytes = spi_audio_get_min_buffer_size(g_pcm);
    fprintf(stderr, "period size '%d', period count '%d', so, spi packet data length '%d', ring buffer size '(%d)*(%d)=(%d)'\n",
        sDefaultSpiAudioConfig.periodSize, sDefaultSpiAudioConfig.periodCount,
        bytes, bytes, sDefaultSpiAudioConfig.periodCount, bytes * sDefaultSpiAudioConfig.periodCount);

    fprintf(stderr, "create file '%s'\n", "/data/spi.pcm");
    file = fopen("/data/spi.pcm", "wb");
    if (!file) {
        fprintf(stderr, "Unable to create file '%s'\n", "/data/spi.pcm");
        return -1;
    }

    //ret = pthread_create(&gSpiWriteThread , NULL, spi_write_dummy_data_run, NULL);

    /* install signal handler and begin capturing */
    //signal(SIGINT, sigint_handler);
    bytes = capture_sample(file, g_pcm);
    printf("Captured %d bytes\n", bytes);

    //pthread_join(spi_write_dummy_data_run, NULL);

    fclose(file);
    spi_audio_close(g_pcm);
    sleep(1);

    goto Loop;

    return 0;
}

unsigned int capture_sample(FILE *file, struct spi_audio *audio)
{
    unsigned char *buffer;
    unsigned int size;
    unsigned int bytes_read = 0;

    size = spi_audio_get_min_buffer_size(audio) * 4;
    buffer = malloc(size);
    if (!buffer) {
        fprintf(stderr, "Unable to allocate %d bytes\n", size);
        return 0;
    }

    printf("Capturing sample: buffer size %d \n", size);

    while (capturing && spi_audio_read(audio, buffer, size) > 0) {
        if (fwrite(buffer, 1, size, file) != size) {
            fprintf(stderr,"Error capturing sample\n");
            break;
        }

        bytes_read += size;

        if (bytes_read > 1024 * 512 * 1) {
            //break;
        }
    }

    free(buffer);

    printf("Capturing sample: bytes_read=%d \n", bytes_read);
    return bytes_read;
}

