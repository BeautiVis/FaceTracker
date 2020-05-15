//
// Created by root on 20-5-12.
//

#ifndef DLIB_ANDROID_APP_BFM_MODEL_H
#define DLIB_ANDROID_APP_BFM_MODEL_H

#include "transform.h"
#include <vector>
#include "jni_primitives.h"
#include "jni_utils.h"
#include "functor.h"
#include <numeric>

using namespace std;

#define N_PICKED_LANDMARK 6
#define N_LANDMARK 68
#define N_ID_PC 99
#define N_EXPR_PC 29

class BfmModel {
public:
	BfmModel() : shapeMu(N_LANDMARK * 3, 0.f),
	             shapeEv(N_ID_PC, 0.f),
	             shapePc(N_LANDMARK * 3, vector<double>(N_ID_PC, 0.f)),
	             exprMu(N_LANDMARK * 3, 0.f),
	             exprEv(N_EXPR_PC, 0.f),
	             exprPc(N_LANDMARK * 3, vector<double>(N_EXPR_PC, 0.f)),
	             _cShape(N_LANDMARK * 3, 0.f),
	             _cExpr(N_LANDMARK * 3, 0.f),
	             _cBlendshape(N_LANDMARK * 2, 0.f),
	             pShapeMu(N_PICKED_LANDMARK * 3, 0.f),
	             pShapePc(N_PICKED_LANDMARK * 3, vector<double>(N_ID_PC, 0.f)),
	             pExprMu(N_PICKED_LANDMARK * 3, 0.f),
	             pExprPc(N_PICKED_LANDMARK * 3,  vector<double>(N_EXPR_PC, 0.f)),
	             _pCShape(N_PICKED_LANDMARK * 3, 0.0),
	             _pCExpr(N_PICKED_LANDMARK * 3, 0.0),
	             _pCBlendshape(N_PICKED_LANDMARK * 3, 0.0)
	             { }
	             
	void pickLandmark() {
        for(int i=0; i<N_PICKED_LANDMARK; i++) {
            int idx = pickedIdx[i];

            pShapeMu[i*3] = shapeMu[idx*3];
            pShapeMu[i*3+1] = shapeMu[idx*3+1];
            pShapeMu[i*3+2] = shapeMu[idx*3+2];

            pExprMu[i*3] = exprMu[idx*3];
            pExprMu[i*3+1] = exprMu[idx*3+1];
            pExprMu[i*3+2] = exprMu[idx*3+2];

            for(int j=0; j<N_ID_PC; j++) {
                pShapePc[i*3][j] = shapePc[idx*3][j];
                pShapePc[i*3+1][j] = shapePc[idx*3+1][j];
                pShapePc[i*3+2][j] = shapePc[idx*3+2][j];
            }

            for(int j=0; j<N_EXPR_PC; j++) {
                pExprPc[i*3][j] = exprPc[idx*3][j];
                pExprPc[i*3+1][j] = exprPc[idx*3+1][j];
                pExprPc[i*3+2][j] = exprPc[idx*3+2][j];
            }
        }
	}             
	             
	PointF *model2Screen() {
    	PointF* array = new PointF[N_PICKED_LANDMARK];
		vector<double> pointsAfterTransform = transformPoints(extParams, _pCBlendshape);
		for(int i=0; i<N_PICKED_LANDMARK; i++) {
			array[i].x = float(pointsAfterTransform[i*3] * intParams[0] / pointsAfterTransform[i*3+2] + intParams[2]);
			array[i].y = float(pointsAfterTransform[i*3+1] * intParams[1] / pointsAfterTransform[i*3+2] + intParams[3]);
		}
		return array;
	}

	void generate() {
		_cShape = coef2object(_shapeCoef, shapeMu, shapePc, shapeEv);
		_cExpr = coef2object(_exprCoef, exprMu, exprPc, exprEv);
		_cBlendshape = pointwiseAdd(_cShape, _cExpr);

        _pCShape = coef2object(_shapeCoef, pShapeMu, pShapePc, shapeEv);
        _pCExpr = coef2object(_exprCoef, pExprMu, pExprPc, exprEv);
        _pCBlendshape = pointwiseAdd(_pCShape, _pCExpr);
	}

	void updateByShape() {
		_pCShape = coef2object(_shapeCoef,pShapeMu, pShapePc, shapeEv);
		_pCBlendshape = pointwiseAdd(_pCShape, _pCExpr);
	}

	void updateByExpr() {
		_pCExpr = coef2object(_exprCoef, pExprMu, pExprPc, exprEv);
		_pCBlendshape = pointwiseAdd(_pCShape, _pCExpr);
	}

	template<typename _Tp>
	const vector<_Tp> generateFaceByShape(const _Tp *const shapeCoef) {
		vector<_Tp> cShape = coef2object(shapeCoef, pShapeMu, pShapePc, shapeEv);
		vector<_Tp> cExpr = vecCast(_pCExpr, (vector<_Tp> *)nullptr);
		return pointwiseAdd(cShape, cExpr);
	}

	template<typename _Tp>
	const vector<_Tp> generateFaceByExpr(const _Tp *const exprCoef) {
		vector<_Tp> cShape = vecCast(_pCShape, (vector<_Tp> *)nullptr);
		vector<_Tp> cExpr = coef2object(exprCoef, pExprMu, pExprPc, exprEv);
		return pointwiseAdd(cShape, cExpr);
	}

	double *getExtParams() { return extParams; }
	double *getIntParams() { return intParams; }
	double *getShapeCoef() { return _shapeCoef; }
	double *getExprCoef()  { return _exprCoef; }

	const vector<double> &getCShape()      const { return _cShape; }
	const vector<double> &getCExpr()       const { return _cExpr; }
	const vector<double> &getCBlendshape() const { return _cBlendshape; }
	const vector<double> &getPCShape()      const { return _pCShape; }
	const vector<double> &getPCExpr()       const { return _pCExpr; }
	const vector<double> &getPCBlendshape() const { return _pCBlendshape; }

public:
    int pickedIdx[N_PICKED_LANDMARK] = {8, 30, 36, 45, 48, 54};

	// Rx Ry Rz Tx Ty Tz
	double extParams[6] = { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };
	// pre-calibration fx fy cx cy
	double intParams[4] = { 2468.247368994031,
					       2456.598297752814,
						   1224.0, 1632.0 };

	double _shapeCoef[N_ID_PC] = { 0.f };
	vector<double> shapeMu;
	vector<double> shapeEv;
	vector<vector<double>> shapePc;

	double _exprCoef[N_EXPR_PC] = { 0.f };
	vector<double> exprMu;
	vector<double> exprEv;
	vector<vector<double>> exprPc;

	vector<double> _cShape;
	vector<double> _cExpr;
	vector<double> _cBlendshape;

    vector<double> pShapeMu;
    vector<vector<double>> pShapePc;
    vector<double> pExprMu;
    vector<vector<double>> pExprPc;
    vector<double> _pCShape;
    vector<double> _pCExpr;
    vector<double> _pCBlendshape;

	jobjectArray landmarks;

private:
	template<typename _Tp>
	vector<_Tp> coef2object(const _Tp *coef, const vector<double> &mu,
	                               const vector<vector<double>> &pc, vector<double> &ev) const
	{
	    vector<_Tp> _mu, _ev;
	    vector<vector<_Tp>> _pc;
	    _mu = vecCast(mu, (vector<_Tp> *)nullptr);
	    _ev = vecCast(ev, (vector<_Tp> *)nullptr);
	    _pc = matCast(pc, (vector<_Tp> *)nullptr);

        vector<_Tp> pM = pointwiseMul(coef, _ev);
        vector<_Tp> M = matMulVec(_pc, pM);
		return pointwiseAdd(_mu, M);
	}
};


#endif //DLIB_ANDROID_APP_BFM_MODEL_H
