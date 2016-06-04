# Copyright (C) 2016 AISpeech, Inc.
#
# Author: caijun.yang <caijun.yang@aispeech.com>
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#      http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

LOCAL_PATH:= $(call my-dir)
include $(CLEAR_VARS)

LOCAL_SHARED_LIBRARIES += \
    libcutils \
    libutils

LOCAL_C_INCLUDES:=

LOCAL_SRC_FILES := \
    spi_audio.c \
    spiaudiotest.c

LOCAL_MODULE := spiaud_test
include $(BUILD_EXECUTABLE)

###################################

include $(CLEAR_VARS)

LOCAL_MODULE:= libspiaudiojni

LOCAL_SRC_FILES:= \
  spi_audio.c \
  spi_audio_jni.c

LOCAL_SHARED_LIBRARIES := \
	libutils liblog libnativehelper

LOCAL_STATIC_LIBRARIES :=

LOCAL_C_INCLUDES += \
	$(JNI_H_INCLUDE)

include $(BUILD_SHARED_LIBRARY)