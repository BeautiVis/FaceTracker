//
// Created by root on 20-5-13.
//

#ifndef DLIB_ANDROID_APP_FUNCTOR_H
#define DLIB_ANDROID_APP_FUNCTOR_H

#include "print.h"
#include <vector>
#include "ceres/ceres.h"
#include "bfm_model.h"
#include "transform.h"
#include "print.h"

using namespace std;

extern BfmModel *g_pBfmModel;
extern jfieldID g_getPointX;
extern jfieldID g_getPointY;

class ExtParamsReprojErr 
{
public:                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                             
	ExtParamsReprojErr(JNIEnv *env) : _env(env) { }
	
    template<typename _Tp>
	bool operator () (const _Tp* const x, _Tp* residuals) const {
		_Tp fx = _Tp(g_pBfmModel->intParams[0]), fy = _Tp(g_pBfmModel->intParams[1]);
		_Tp cx = _Tp(g_pBfmModel->intParams[2]), cy = _Tp(g_pBfmModel->intParams[3]);

		const vector<double> _fp_shape = g_pBfmModel->getPCBlendshape();

		const vector<_Tp> fp_shape = transformPoints(x, _fp_shape);

		for(unsigned int i=0; i<N_PICKED_LANDMARK; i++)
		{
		    int idx = g_pBfmModel->pickedIdx[i];
			jobject point = _env->GetObjectArrayElement(g_pBfmModel->landmarks, static_cast<jsize>(idx));

			_Tp u = fx * fp_shape[i*3] / fp_shape[i*3+2] + cx;
			_Tp v = fy * fp_shape[i*3+1] / fp_shape[i*3+2] + cy;

			residuals[i*2] = _Tp(_env->GetIntField(point, g_getPointX)) - u;
			residuals[i*2+1] = _Tp(_env->GetIntField(point, g_getPointY)) - v;
		}
		return true;
	}

	static ceres::CostFunction *create(JNIEnv *env) {
		return (new ceres::AutoDiffCostFunction<ExtParamsReprojErr, N_PICKED_LANDMARK*2, 6>(
			new ExtParamsReprojErr(env)));
	}

private:
	JNIEnv *_env;
};


class shapeCoefReprojErr {
public:
    shapeCoefReprojErr(JNIEnv *env) : _env(env) { }

    template<typename _Tp>
    bool operator () (const _Tp* const shapeCoef, _Tp* residuals) const {
        _Tp fx = _Tp(g_pBfmModel->intParams[0]), fy = _Tp(g_pBfmModel->intParams[1]);
        _Tp cx = _Tp(g_pBfmModel->intParams[2]), cy = _Tp(g_pBfmModel->intParams[3]);

        const vector<_Tp> _fp_shape = g_pBfmModel->generateFaceByShape(shapeCoef);
        const double *extrinsic_params = g_pBfmModel->getExtParams();
        _Tp *extrinsic_params_ = new _Tp[6];
        for(int i=0; i<6; i++)
            extrinsic_params_[i] = (_Tp)(extrinsic_params[i]);

        const vector<_Tp> fp_shape = transformPoints(extrinsic_params_, _fp_shape);

        for(unsigned int i=0; i<N_PICKED_LANDMARK; i++) {
            int idx = g_pBfmModel->pickedIdx[i];
            jobject point = _env->GetObjectArrayElement(g_pBfmModel->landmarks, static_cast<jsize>(idx));
            _Tp u = fx * fp_shape[i*3] / fp_shape[i*3+2] + cx;
            _Tp v = fy * fp_shape[i*3+1] / fp_shape[i*3+2] + cy;
            residuals[i*2] = _Tp(_env->GetIntField(point, g_getPointX)) - u;
            residuals[i*2+1] = _Tp(_env->GetIntField(point, g_getPointY)) - v;
        }
        return true;
    }

    static ceres::CostFunction *create(JNIEnv *env) {
        return (new ceres::AutoDiffCostFunction<shapeCoefReprojErr, N_PICKED_LANDMARK * 2, N_ID_PC>(
                new shapeCoefReprojErr(env)));
    }

private:
    JNIEnv *_env;
};


class exprCoefReprojErr {
public:
    exprCoefReprojErr(JNIEnv *env) : _env(env) { }

    template<typename _Tp>
    bool operator () (const _Tp* const exprCoef, _Tp* residuals) const {
        _Tp fx = _Tp(g_pBfmModel->intParams[0]), fy = _Tp(g_pBfmModel->intParams[1]);
        _Tp cx = _Tp(g_pBfmModel->intParams[2]), cy = _Tp(g_pBfmModel->intParams[3]);

        const vector<_Tp> fp_shape_ = g_pBfmModel->generateFaceByExpr(exprCoef);

        const double *extrinsic_params = g_pBfmModel->getExtParams();
        _Tp *extrinsic_params_ = new _Tp[6];
        for(int i=0; i<6; i++)
            extrinsic_params_[i] = (_Tp)(extrinsic_params[i]);

        const vector<_Tp> fp_shape = transformPoints(extrinsic_params_, fp_shape_);

        for(int i=0; i<N_PICKED_LANDMARK; i++) {
            int idx = g_pBfmModel->pickedIdx[i];
            jobject point = _env->GetObjectArrayElement(g_pBfmModel->landmarks, static_cast<jsize>(idx));
           _Tp u = fx * fp_shape[i*3] / fp_shape[i*3+2] + cx;
            _Tp v = fy * fp_shape[i*3+1] / fp_shape[i*3+2] + cy;
            residuals[i*2] = _Tp(_env->GetIntField(point, g_getPointX)) - u;
            residuals[i*2+1] = _Tp(_env->GetIntField(point, g_getPointY)) - v;
        }
        return true;
    }

    static ceres::CostFunction *create(JNIEnv *env) {
        return (new ceres::AutoDiffCostFunction<exprCoefReprojErr, N_PICKED_LANDMARK*2, 29>(
                new exprCoefReprojErr(env)));
    }

private:
    JNIEnv *_env;
};


class shapeCoefRegTerm {
public:
    shapeCoefRegTerm() {}
    template<typename T>
    bool operator () (const T* const shape_coef, T* residuals) const {
        for(int i=0; i<99; i++)
            residuals[i] = T(u) * shape_coef[i];
        return true;
    }

    static ceres::CostFunction *create() {
        return (new ceres::AutoDiffCostFunction<shapeCoefRegTerm, 99, 99>(
                new shapeCoefRegTerm()));
    }
private:
    const double u = 0.003;
};

class exprCoefRegTerm {
public:
    exprCoefRegTerm() {}
    template<typename T>
    bool operator () (const T* const expr_coef, T* residuals) const {
        for(int i=0; i<29; i++)
            residuals[i] = T(u) * expr_coef[i];
        return true;
    }

    static ceres::CostFunction *create() {
        return (new ceres::AutoDiffCostFunction<exprCoefRegTerm, 29, 29>(
                new exprCoefRegTerm()));
    }
private:
    const double u = 0.01;
};




#endif //DLIB_ANDROID_APP_FUNCTOR_H
