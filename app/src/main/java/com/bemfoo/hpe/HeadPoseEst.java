package com.bemfoo.hpe;

import android.graphics.Point;
import android.graphics.PointF;
import android.util.Log;

import java.io.BufferedReader;
import java.io.InputStream;
import java.io.InputStreamReader;


public class HeadPoseEst {
    private static final String TAG = "HPE";

    /* Load ceres support library, which is connected with jni_hpe.cpp */

    static {
        try {
            System.loadLibrary("jni_hpe");
            Log.d(TAG, "library loaded successfully");
        } catch (UnsatisfiedLinkError e) {
            Log.e(TAG, "library not found");
        }
    }


    /**
     * This function is to close the optimality using ceres.
     *
     * @param landmarks 2d point got from dlib
     */
    public static native PointF[] jniSolve(double[] extParams, Point[] landmarks);

}
