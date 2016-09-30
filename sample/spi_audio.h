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

#ifndef __SPIAUD_H
#define __SPIAUD_H

#if defined(__cplusplus)
extern "C" {
#endif

#define SPI_AUDIO_DEV_PATH      "/dev/spiaud"
#define SPI_AUDIO_CHANNEL_NUM   8

/* Bit formats */
enum spi_audio_format {
    SPI_AUDIO_FORMAT_S16_LE = 0,
    SPI_AUDIO_FORMAT_S32_LE,
    SPI_AUDIO_FORMAT_S8,
    SPI_AUDIO_FORMAT_S24_LE,

    SPI_AUDIO_FORMAT_MAX,
};

/* Configuration for a stream */
struct spi_audio_config {
    unsigned int channels;
    unsigned int rate;
    unsigned int periodSize;
    unsigned int periodCount;
    enum spi_audio_format format;
};

/* Paramter for the SPI slaver */
struct spi_audio_channels_gain {
    unsigned char gainDb[SPI_AUDIO_CHANNEL_NUM];
};

struct spi_audio {
    int fd;
    int running:1;
    int underruns;
    struct spi_audio_config config;
};

unsigned int spi_audio_get_min_buffer_size(struct spi_audio *pAudio);
int spi_audio_write(struct spi_audio *pAudio, const unsigned char *pData, int count);
int spi_audio_read(struct spi_audio *pAudio, unsigned char *pData, int count);
int spi_audio_close(struct spi_audio *pAudio);
struct spi_audio *spi_audio_open(struct spi_audio_config *pConfig);
int spi_audio_is_ready(struct spi_audio *pAudio);
int spi_audio_set_channel_gains(struct spi_audio *pAudio, struct spi_audio_channels_gain *pGain);
int spi_audio_set_em_mode(struct spi_audio *pAudio, unsigned char bActive);
int spi_audio_set_oneshot_param(struct spi_audio *pAudio, unsigned short value);

/* IOCTL commands */
#define SPI_IOC_MAGIC           'k'

/* Must set the config before getting the audio stream  */
#define SPI_IOC_WR_INIT             _IOW(SPI_IOC_MAGIC, 5, struct spi_audio_config *)

/* Set/Reset the audio codec paramters to the SPI slaver */
#define SPI_IOC_WR_CHANNELS_GAIN    _IOW(SPI_IOC_MAGIC, 6, struct spi_audio_channels_gain *)

/* Set/Reset EM mode to the SPI slaver */
#define SPI_IOC_WR_EM_MODE          _IOW(SPI_IOC_MAGIC, 7, __u8)

/* Set oneshot param to the SPI slaver */
#define SPI_IOC_WR_ONESHOT_PARAM    _IOW(SPI_IOC_MAGIC, 8, __u16)

#if defined(__cplusplus)
}  /* extern "C" */
#endif

#endif
