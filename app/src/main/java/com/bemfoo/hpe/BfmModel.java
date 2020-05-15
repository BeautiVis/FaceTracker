package com.bemfoo.hpe;

import android.support.annotation.Keep;
import android.support.annotation.NonNull;
import android.util.Log;

public class BfmModel {
    private static final String TAG = "BFM";

    static {
        try {
            System.loadLibrary("jni_hpe");
            Log.d(TAG, "library loaded successfully");
        } catch (UnsatisfiedLinkError e) {
            Log.e(TAG, "library not found");
        }
    }


    public static native void jniLoadData(String path);

}
