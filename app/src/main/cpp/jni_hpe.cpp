#include <jni.h>
#include <string>
#include <cmath>
#include <iostream>
#include <ceres/ceres.h>
#include "functor.h"
#include "bfm_model.h"
#include "print.h"
#include "jni_utils.h"
#include "jni_primitives.h"

using namespace std;

#define BFM_JNI_METHOD(METHOD_NAME) Java_com_bemfoo_hpe_BfmModel_##METHOD_NAME
#define HPE_JNI_METHOD(METHOD_NAME) Java_com_bemfoo_hpe_HeadPoseEst_##METHOD_NAME

BfmModel *g_pBfmModel;

jclass g_classPointF;
jclass g_classPoint;
jfieldID g_getPointX;
jfieldID g_getPointY;
jmethodID g_initPointF;

ceres::Solver::Options g_options;
ceres::Solver::Summary g_summary;


void solveExtParams(JNIEnv *env) {
	LOGI("Solve external parameters");

	ceres::Problem problem;
	double *extParams = g_pBfmModel->getExtParams();
	ceres::CostFunction *costFunction = ExtParamsReprojErr::create(env);
	problem.AddResidualBlock(costFunction, nullptr, extParams);
	ceres::Solve(g_options, &problem, &g_summary);
	LOGI("%s", g_summary.BriefReport().c_str());
}


void solveShapeCoef(JNIEnv *env) {
    ceres::Problem problem;
    double *shape_coef = g_pBfmModel->getShapeCoef();
    ceres::CostFunction *cost_function = shapeCoefReprojErr::create(env);
    ceres::CostFunction *reg_term = shapeCoefRegTerm::create();
    problem.AddResidualBlock(cost_function, nullptr, shape_coef);
    problem.AddResidualBlock(reg_term, nullptr, shape_coef);
    ceres::Solve(g_options, &problem, &g_summary);
    LOGI("%s", g_summary.BriefReport().c_str());
    g_pBfmModel->updateByShape();
}


void solveExprCoef(JNIEnv *env) {
    ceres::Problem problem;
    double *expr_coef = g_pBfmModel->getExprCoef();
    ceres::CostFunction *cost_function = exprCoefReprojErr::create(env);
    ceres::CostFunction *reg_term = exprCoefRegTerm::create();
    problem.AddResidualBlock(cost_function, nullptr, expr_coef);
    problem.AddResidualBlock(reg_term, nullptr, expr_coef);
    ceres::Solve(g_options, &problem, &g_summary);
    LOGI("%s", g_summary.BriefReport().c_str());
    g_pBfmModel->updateByExpr();
}


JNIEXPORT jint JNI_OnLoad(JavaVM* vm, void* reserved) {
    LOGI("JNI_OnLoad enter");
    JNIEnv* env;
    vm->GetEnv((void**)&env, JNI_VERSION_1_6);

    g_classPointF = env->FindClass("android/graphics/PointF");
    if(g_classPointF == NULL) { LOGE("Find PointF failed"); }

    g_classPoint = env->FindClass("android/graphics/Point");
    if(g_classPoint == NULL) { LOGE("Find Point failed"); }

    g_getPointX = env->GetFieldID(g_classPoint, "x", "I");
    if(g_getPointX == NULL) { LOGE("Get Point X failed"); }

    g_getPointY = env->GetFieldID(g_classPoint, "y", "I");
    if(g_getPointX == NULL) { LOGE("Get Point Y failed"); }

    g_initPointF = env->GetMethodID(g_classPointF, "<init>", "(FF)V");
    if(g_initPointF == NULL) { LOGE("Get PointF init failed"); }

	g_options.max_num_iterations = 100;
	g_options.num_threads = 8;
	g_options.minimizer_progress_to_stdout = false;

    LOGI("JNI_OnLoad exit");
    return JNI_VERSION_1_6;
}


extern "C"
JNIEXPORT void JNICALL
BFM_JNI_METHOD(jniLoadData)(JNIEnv* env, jclass thiz, jstring jFaceDataPath) {
	LOG(INFO) << "jniBfmModelInit";
	if(!g_pBfmModel) g_pBfmModel = new BfmModel();
	std::string path = convertJStrToString(env, jFaceDataPath);
	if (fileExists(path)) {
		std::ifstream in;
		in.open(path);
		if(!in) {
			LOGE("File read faield");
			return;
		}
		for(int i=0; i<N_LANDMARK * 3; i++) in >> g_pBfmModel->shapeMu[i];
		for(int i=0; i<N_ID_PC; i++) in >> g_pBfmModel->shapeEv[i];
		for(int i=0; i<N_LANDMARK * 3; i++)
			for(int j=0; j<N_ID_PC; j++)
				in >> g_pBfmModel->shapePc[i][j];
		for(int i=0; i<N_LANDMARK * 3; i++) in >> g_pBfmModel->exprMu[i];
		for(int i=0; i<N_EXPR_PC; i++) in >> g_pBfmModel->exprEv[i];
		for(int i=0; i<N_LANDMARK * 3; i++)
			for(int j=0; j<N_EXPR_PC; j++)
				in >> g_pBfmModel->exprPc[i][j];
		in.close();
	} else {
		LOGE("Not exist %s", path.c_str());
	}

	g_pBfmModel->pickLandmark();
	g_pBfmModel->generate();
}


extern "C"
JNIEXPORT jobjectArray JNICALL
HPE_JNI_METHOD(jniSolve)(JNIEnv *env, jclass thiz, jdoubleArray extParams, jobjectArray landmarks) {
    LOGI("Solve head pose estimation");
    jdouble *x = env->GetDoubleArrayElements(extParams, nullptr);

	g_pBfmModel->landmarks = landmarks;

	solveExtParams(env);
	solveShapeCoef(env);
	solveExprCoef(env);

	PointF *pointArray = g_pBfmModel->model2Screen();

    g_classPointF = env->FindClass("android/graphics/PointF");
	jobjectArray results = env->NewObjectArray(N_PICKED_LANDMARK, g_classPointF, NULL);

	for(unsigned int i=0; i<N_PICKED_LANDMARK; i++) {
		jobject point = env->NewObject(g_classPointF, g_initPointF, pointArray[i].x, pointArray[i].y);
		env->SetObjectArrayElement(results, i, point);
		env->DeleteLocalRef(point);
	}

	for(int i=0; i<6; i++)
        x[i] = g_pBfmModel->extParams[i];
    env->ReleaseDoubleArrayElements(extParams, x, 0);

	return results;
}