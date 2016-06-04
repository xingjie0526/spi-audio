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

/*
 * Native implementation for the SpiAudioRecorder class.
 */

#define LOG_TAG "SPI_AUDIO_JNI"
#define LOG_NDEBUG 0

#include <errno.h>
#include <inttypes.h>
#include <log/log.h>

#include <jni.h>
#include <JNIHelp.h>
#include "spi_audio.h"

static struct spi_audio *g_pSpiAudio = NULL;

static jint spiaudio_native_setup(JNIEnv *env, jclass clazz,
                    jint sampleRateInHertz, jint channelNum, jint audioFormat,
                    jint periodSize, jint periodCount) {
    struct spi_audio_config audioConfig;
    audioConfig.channels = channelNum;
    audioConfig.rate = sampleRateInHertz;
    audioConfig.periodSize = periodSize;
    audioConfig.periodCount = periodCount;
    audioConfig.format = audioFormat;

    if (g_pSpiAudio != NULL) {
        ALOGE("SPI Audio has been open!");
        return (jint)-1;
    }

    ALOGD("native_setup(%d, %d, %d, %d , %d)",
            sampleRateInHertz, channelNum, audioFormat, periodSize, periodCount);
    g_pSpiAudio = spi_audio_open(&audioConfig);
    if (!g_pSpiAudio || !spi_audio_is_ready(g_pSpiAudio)) {
        ALOGE("Unable to open SPI Audio device!");
        spi_audio_close(g_pSpiAudio);
        g_pSpiAudio = NULL;
        return (jint)-1;
    }

    return 0;
}

static void spiaudio_native_release(JNIEnv *env, jclass clazz) {
    ALOGD("native_release()");
    if (g_pSpiAudio) {
        spi_audio_close(g_pSpiAudio);
        g_pSpiAudio = NULL;
    }
}

static jint spiaudio_native_read(JNIEnv *env, jclass clazz, jbyteArray javaAudioData,
                    jint offsetInBytes, jint sizeInBytes) {
    jbyte* recordBuff = NULL;
    int    readSize;

    //ALOGD("native_read()");
    if (!g_pSpiAudio || !spi_audio_is_ready(g_pSpiAudio)) {
        ALOGE("SPI Audio device is not ready!");
        return (jint)-1;
    }

    if (!javaAudioData) {
        ALOGE("Invalid Java array to store recorded audio, can't record");
        return 0;
    }

    if (sizeInBytes < spi_audio_get_min_buffer_size(g_pSpiAudio)) {
        ALOGE("Buffer size (%d) is less min buffer size (%d), can't record", sizeInBytes, spi_audio_get_min_buffer_size(g_pSpiAudio));
        return 0;
    }

    recordBuff = (jbyte *)(*env)->GetByteArrayElements(env, javaAudioData, NULL);
    if (recordBuff == NULL) {
        ALOGE("Error retrieving destination for recorded audio data, can't record");
        return 0;
    }

    readSize = spi_audio_read(g_pSpiAudio, recordBuff + offsetInBytes, sizeInBytes);
    (*env)->ReleaseByteArrayElements(env, javaAudioData, recordBuff, 0);
    if (readSize < 0) {
        ALOGE("Invalid operation");
    }

    return (jint)readSize;
}

static jint spiaudio_native_set_channels_gain(JNIEnv *env, jclass clazz, jbyteArray javaChannelsGainData,
                    jint channelNum) {
    jbyte* channelsGainData = NULL;
    struct spi_audio_channels_gain  channelsGain;
    int i, errCode;

    ALOGD("native_set_channels_gain()");
    if (!g_pSpiAudio || !spi_audio_is_ready(g_pSpiAudio)) {
        ALOGE("SPI Audio device is not ready!");
        return -1;
    }

    channelsGainData = (jbyte *)(*env)->GetByteArrayElements(env, javaChannelsGainData, NULL);
    if (channelsGainData == NULL) {
        ALOGE("Error retrieving destination for channels gain data");
        return -1;
    }

    memset(channelsGain.gainDb, 0xff, sizeof(channelsGain));
    for (i=0; i<channelNum && i<SPI_AUDIO_CHANNEL_NUM; i++) {
        channelsGain.gainDb[i] = channelsGainData[i];
    }

    errCode = spi_audio_set_channels_gain(g_pSpiAudio, &channelsGain);

    (*env)->ReleaseByteArrayElements(env, javaChannelsGainData, channelsGainData, 0);

    return (jint)errCode;
}

static JNINativeMethod methods[] = {
    // name, signature, function
    { "native_setup", "(IIIII)I", spiaudio_native_setup },
    { "native_release", "()V", spiaudio_native_release },
    { "native_read", "([BII)I", spiaudio_native_read },
    { "native_set_channels_gain", "([BI)I", spiaudio_native_set_channels_gain },
};

int registerNatives(JNIEnv *env) {
/*
    return jniRegisterNativeMethods(
            env, "com/aispeech/audio/SpiAudioRecorder",
            methods, sizeof(methods) / sizeof(JNINativeMethod));
*/
    jclass jcls = (*env)->FindClass(env, "com/aispeech/audio/SpiAudioRecorder");
    if (!jcls) return JNI_ERR;

    return (*env)->RegisterNatives(env, jcls, methods, sizeof(methods)/sizeof(methods[0]));
}

jint JNI_OnLoad(JavaVM *vm, void *reserved) {
    JNIEnv *env = NULL;

    if ((*vm)->GetEnv(vm, (void **) &env, JNI_VERSION_1_4) != JNI_OK) {
        return JNI_ERR;
    }

    if (registerNatives(env)) {
        ALOGE("failed to register native methods for 'com/aispeech/audio/SpiAudioRecorder'");
        return JNI_ERR;
    }

    return JNI_VERSION_1_4;
}
