//
// Created by root on 20-5-12.
//

#ifndef DLIB_ANDROID_APP_TRANSFORM_H
#define DLIB_ANDROID_APP_TRANSFORM_H

#define _USE_MATH_DEFINES
#include <cmath>
#include <vector>

using namespace std;

template<typename _Tp> inline
vector<vector<_Tp>> rotMatrix(const _Tp &yaw, const _Tp &roll, const _Tp &pitch) {
    _Tp c1 = cos(yaw),   s1 = sin(yaw);
    _Tp c2 = cos(pitch), s2 = sin(pitch);
    _Tp c3 = cos(roll),  s3 = sin(roll);
    vector<vector<_Tp>> R{ {c2 * c1, s3 * s2 * c1 - c3 * s1, c3 * s2 * c1 + s3 * s1},
                           {c2 * s1, s3 * s2 * s1 + c3 * c1, c3 * s2 * s1 - s3 * c1},
                           {-s2, s3 * c2, c3 * c2} };
    return R;
}

template<typename _Tp, typename _Ep> inline
vector<_Tp> transformPoints(const _Tp *extParams, const vector<_Ep> &bPoints) {
    unsigned long nVertices = bPoints.size() / 3;
    vector<_Tp> aPoints(bPoints.size());
    vector<vector<_Tp>> R = rotMatrix(extParams[0], extParams[1], extParams[2]);
    for(unsigned int i=0; i<nVertices; i++) {
        aPoints[i*3] = (_Tp)bPoints[i*3] * R[0][0]
                     + (_Tp)bPoints[i*3+1] * R[0][1]
                     + (_Tp)bPoints[i*3+2] * R[0][2]
                     + extParams[3];
        aPoints[i*3+1] = (_Tp)bPoints[i*3] * R[1][0]
                       + (_Tp)bPoints[i*3+1] * R[1][1]
                       + (_Tp)bPoints[i*3+2] * R[1][2]
                       + extParams[4];
        aPoints[i*3+2] = (_Tp)bPoints[i*3] * R[2][0]
                       + (_Tp)bPoints[i*3+1] * R[2][1]
                       + (_Tp)bPoints[i*3+2] * R[2][2]
                       + extParams[5];
    }
    return aPoints;
}

#endif //DLIB_ANDROID_APP_TRANSFORM_H
