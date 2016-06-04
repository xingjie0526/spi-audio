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
#include <fcntl.h>
#include <stdarg.h>
#include <string.h>
#include <errno.h>
#include <unistd.h>
#include <poll.h>

#include <sys/ioctl.h>
#include <sys/mman.h>
#include <sys/time.h>
#include <limits.h>

#include <linux/ioctl.h>
#include "spi_audio.h"

#include <utils/Log.h>

#define LOG_TAG  "SPI_AUDIO"

#define SPI_AUDIO_SAVE_RAW_FILE     0
static FILE *gPcmRawFile = NULL;
static int gFileIndex = 0;

static struct spi_audio_config sDefaultSpiAudioConfig = {
    .channels = 8,
    .rate = 16000,
    .periodSize = 320 /* (sample_rate * BUFF_DURATION_MS) / 1000 = 16000 * 20 / 1000 = 320 */
    .periodCount = 10,
    .format = SPI_AUDIO_FORMAT_S16_LE,
};

static struct spi_audio_channels_gain sDefaultSpiAudioChannelsGain = {
    .gainDb = {60, 60, 60, 60, 60, 60, 60, 0},
};

static struct spi_audio gSpiAudio = {
    .fd = -1,
};

static unsigned int spi_audio_format_to_bits(enum spi_audio_format format) {
    switch (format) {
    case SPI_AUDIO_FORMAT_S32_LE:
        return 32;
    case SPI_AUDIO_FORMAT_S24_LE:
        return 24;
    default:
    case SPI_AUDIO_FORMAT_S16_LE:
        return 16;
    };
}

static unsigned int spi_audio_frames_to_bytes(struct spi_audio *pAudio, unsigned int frames) {
    return frames * pAudio->config.channels * (spi_audio_format_to_bits(pAudio->config.format) >> 3);
}

unsigned int spi_audio_get_min_buffer_size(struct spi_audio *pAudio) {
    return spi_audio_frames_to_bytes(pAudio, pAudio->config.periodSize);
}

int spi_audio_read(struct spi_audio *pAudio, unsigned char *pData, int count) {
    int frameSize = spi_audio_frames_to_bytes(pAudio, 1);
    int maxBufferSize = spi_audio_frames_to_bytes(pAudio, pAudio->config.periodSize * pAudio->config.periodCount);
    int readBytes = 0;
    int bytes = 0;
    int ret = -1;

    if (count % frameSize != 0) {
        count = (count / frameSize) * frameSize;
    }

    //ALOGD("spi_audio_read()");
    while (readBytes < count) {
        bytes = (count - readBytes) < maxBufferSize ? (count - readBytes) : maxBufferSize;
        ret = read(pAudio->fd, &pData[readBytes], bytes);
        if (ret <= 0) {
            ALOGD("spi_audio_read() read(bytes=%d) return %d", bytes, ret);
        }
        
        if (ret < 0 ) {
            ALOGE("spi_audio_read() cannot read stream data");
            break;
        }
        readBytes = readBytes + ret;
    }

    if ((!gPcmRawFile) && (SPI_AUDIO_SAVE_RAW_FILE)) {
        char filePath[50] = {0};
        sprintf(filePath, "/sdcard/aispeech/spiaudio_%d.pcm", (gFileIndex % 10));
        gFileIndex++;
        gPcmRawFile = fopen(filePath, "wb");
        if (gPcmRawFile) {
            ALOGE("spi_audio_read(), fopen(%s) success!", filePath);
        } else {
            ALOGE("spi_audio_read(), fopen(%s) fail!!!", filePath);
        }
    }

    if (gPcmRawFile) {
        //ALOGD("spi_audio_read(count=%d), fwrite(%d)", count, readBytes);
        fwrite(pData, 1, readBytes, gPcmRawFile);
    }

    //ALOGD("spi_audio_read() count=%d, readBytes=%d", count, readBytes);
    return readBytes > 0 ? readBytes : ret;
}

int spi_audio_close(struct spi_audio *pAudio) {
    ALOGD("spi_audio_close()");
    if (pAudio->fd >= 0) {
        close(pAudio->fd);
    }

    pAudio->fd = -1;

    if (gPcmRawFile) {
        ALOGE("spi_audio_close(), fclose(/sdcard/aispeech/spiaudio_%d.pcm) !", (gFileIndex-1) % 10);
        fclose(gPcmRawFile);
        gPcmRawFile = NULL;
    }

    return 0;
}

int spi_audio_set_channels_gain(struct spi_audio *pAudio, struct spi_audio_channels_gain *pGain) {
    int retCode = 0;
    ALOGD("spi_audio_set_channels_gain()");
    if (pAudio->fd < 0) {
        ALOGE("Device '%s' has been close!", SPI_AUDIO_DEV_PATH);
    }
    retCode = ioctl(pAudio->fd, SPI_IOC_WR_CHANNELS_GAIN, pGain);
    ALOGD("spi_audio_set_channels_gain() Exit");
    return retCode;
}

int spi_audio_set_em_mode(struct spi_audio *pAudio, unsigned char flag) {
    unsigned char  value = flag;
    int retCode = 0;
    ALOGD("spi_audio_set_em_mode()");
    if (pAudio->fd < 0) {
        ALOGE("Device '%s' has been close!", SPI_AUDIO_DEV_PATH);
    }
    retCode = ioctl(pAudio->fd, SPI_IOC_WR_EM_MODE, &value);
    ALOGD("spi_audio_set_em_mode() %d Exit", retCode);
    return retCode;
}

struct spi_audio *spi_audio_open(struct spi_audio_config *pConfig) {
    struct spi_audio *pAudio;

    ALOGD("spi_audio_open()");
    if (gSpiAudio.fd >= 0) {
        ALOGE("spi_audio_open() device '%s' has been open", SPI_AUDIO_DEV_PATH);
        return NULL;
    }

    pAudio = &gSpiAudio;
    pAudio->config = (pConfig != NULL) ? *pConfig : sDefaultSpiAudioConfig;
    pAudio->fd = open(SPI_AUDIO_DEV_PATH, O_RDWR);
    if (pAudio->fd < 0) {
        ALOGE("spi_audio_open() cannot open device '%s'", SPI_AUDIO_DEV_PATH);
        return NULL;
    }

    ALOGD("spi_audio_open() Exit");
    return pAudio;
}

int spi_audio_is_ready(struct spi_audio *pAudio) {
    return pAudio->fd >= 0;
}



