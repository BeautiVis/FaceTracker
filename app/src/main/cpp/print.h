//
// Created by root on 20-5-13.
//

#ifndef DLIB_ANDROID_APP_PRINT_H
#define DLIB_ANDROID_APP_PRINT_H

#include <android/log.h>

/* Logger used for debugging */
#define TAG        "HPE-CPP"
#define LOGD(...)  __android_log_print(ANDROID_LOG_DEBUG, TAG,__VA_ARGS__)
#define LOGI(...)  __android_log_print(ANDROID_LOG_INFO,  TAG,__VA_ARGS__)
#define LOGW(...)  __android_log_print(ANDROID_LOG_WARN,  TAG,__VA_ARGS__)
#define LOGE(...)  __android_log_print(ANDROID_LOG_ERROR, TAG,__VA_ARGS__)
#define LOGF(...)  __android_log_print(ANDROID_LOG_FATAL, TAG,__VA_ARGS__)

#endif //DLIB_ANDROID_APP_PRINT_H
