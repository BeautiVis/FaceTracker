//
// Created by root on 20-5-13.
//

#ifndef DLIB_ANDROID_APP_JNI_UTILS_H
#define DLIB_ANDROID_APP_JNI_UTILS_H

#include "jni.h"
#include <sstream>
#include <unistd.h>
#include <fstream>
#include <iostream>
#include <string>
#include <sys/stat.h>
#include <unistd.h>
#include <vector>


template<typename L, typename R>
std::vector<R> vecCast(const std::vector<L> &left, std::vector<R>*) {
    std::vector<R> right;
    std::transform(
        left.begin(), left.end(),
        std::insert_iterator<std::vector<R>>(right, right.begin()),
        [](L l)->R {return (R)l; }
    );
    return std::move(right);
}

template<typename L, typename R>
std::vector<std::vector<R>> matCast(const std::vector<L> &left, std::vector<R>*) {
    int height = left.size(), width = left[0].size();
    std::vector<std::vector<R>> right(height, std::vector<R>(width));
    for(int i=0; i<height; i++)
        for(int j=0; j<width; j++)
            right[i][j] = (R)left[i][j];
    return std::move(right);
}


template<typename _Tp>
vector<_Tp> matMulVec(const std::vector<std::vector<_Tp>> &mat,
                            const std::vector<_Tp> &vec) {
    int size = mat.size();
    std::vector<_Tp> res(size);
    for(unsigned int i=0; i<size; i++) {
        res[i] = std::inner_product(std::begin(mat[i]), std::end(mat[i]),
                                    std::begin(vec), (_Tp)0.0);
    }
    return std::move(res);
}

template<typename _Tp>
vector<_Tp> pointwiseMul(const std::vector<_Tp> &a, const std::vector<_Tp> &b) {
    int len = a.size();
    std::vector<_Tp> res(len);
    for(unsigned int i=0; i<len; i++)
        res[i] = a[i] * b[i];
    return std::move(res);
}


template<typename _Tp>
vector<_Tp> pointwiseMul(const _Tp *a, const std::vector<_Tp> &b) {
    int len = b.size();
    std::vector<_Tp> res(len);
    for(unsigned int i=0; i<len; i++)
        res[i] = a[i] * b[i];
    return std::move(res);
}

template<typename _Tp>
vector<_Tp> pointwiseAdd(const std::vector<_Tp> &a, const std::vector<_Tp> &b) {
    int len = a.size();
    std::vector<_Tp> res(len);
    for(unsigned int i=0; i<len; i++)
        res[i] = a[i] + b[i];
    return std::move(res);
}


char* convertJStrToCStr(JNIEnv* env, jstring lString) {
  const char* lStringTmp;
  char* pstring;

  lStringTmp = env->GetStringUTFChars(lString, NULL);
  if (lStringTmp == NULL)
    return NULL;

  jsize lStringLength = env->GetStringUTFLength(lString);

  pstring = (char*)malloc(sizeof(char) * (lStringLength + 1));
  strcpy(pstring, lStringTmp);

  env->ReleaseStringUTFChars(lString, lStringTmp);

  return pstring;
}

std::string convertJStrToString(JNIEnv* env, jstring lString) {
  const char* lStringTmp;
  std::string str;

  lStringTmp = env->GetStringUTFChars(lString, NULL);
  if (lStringTmp == NULL)
    return NULL;

  str = lStringTmp;

  env->ReleaseStringUTFChars(lString, lStringTmp);

  return str;
}

bool fileExists(const char* name) {
  std::ifstream ifs(name);
  return ifs.good();
}

bool dirExists(const char* name) {
  struct stat file_info;
  if (stat(name, &file_info) != 0)
    return false;
  return (file_info.st_mode & S_IFDIR) != 0;
}

bool fileExists(const std::string& name) { return fileExists(name.c_str()); }

bool dirExists(const std::string& name) { return dirExists(name.c_str()); }

#endif //DLIB_ANDROID_APP_JNI_UTILS_H
