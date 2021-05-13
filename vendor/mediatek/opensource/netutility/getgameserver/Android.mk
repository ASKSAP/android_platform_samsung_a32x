# Copyright (C) 2014 The Android Open Source Project
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

LOCAL_PATH := $(call my-dir)

###
### get game server daemon.
###
include $(CLEAR_VARS)

LOCAL_CLANG := true
# LOCAL_CFLAGS :=  -Wall
LOCAL_MODULE := getgameserver

# Bug: http://b/29823425 Disable -Wvarargs for Clang update to r271374
LOCAL_CFLAGS +=  -Wno-varargs

ifeq ($(TARGET_ARCH), x86)
ifneq ($(TARGET_PRODUCT), gce_x86_phone)
        LOCAL_CFLAGS += -D NETLINK_COMPAT32
endif
endif

ifneq ($(filter userdebug eng,$(TARGET_BUILD_VARIANT)),)
    LOCAL_CFLAGS += -DMTK_DEBUG
endif

LOCAL_SHARED_LIBRARIES := \
        libcutils \
        liblog \
        libforkexecwrap \
        libutils \
        libpcap


LOCAL_SRC_FILES := \
        addr_hash.c \
        hash.c \
        util.c  \
        main.c

LOCAL_INIT_RC := getgameserver.rc

LOCAL_PROPRIETARY_MODULE := true

LOCAL_MODULE_OWNER := mtk

include $(MTK_EXECUTABLE)





include $(CLEAR_VARS)
###
### test demo.
###


LOCAL_CLANG := true
LOCAL_MODULE := testgameserver


LOCAL_CFLAGS +=  -Wno-varargs

LOCAL_SHARED_LIBRARIES := \
        libcutils \
        liblog \
        libutils 


LOCAL_SRC_FILES := \
        test.c 



LOCAL_PROPRIETARY_MODULE := true

LOCAL_MODULE_OWNER := mtk

include $(MTK_EXECUTABLE)


