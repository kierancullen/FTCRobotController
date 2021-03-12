package org.firstinspires.ftc.teamcode;

import android.app.Activity;
import android.content.Context;
import android.content.SharedPreferences;

public class PreconfigStorage {

    SharedPreferences sharedPref;

    public PreconfigStorage(Context appContext) {
        sharedPref = ((Activity)appContext).getPreferences(Context.MODE_PRIVATE);
    }

    public void write(String key, float value) {
        SharedPreferences.Editor editor = sharedPref.edit();
        editor.putFloat(key, value);
        editor.commit();
    }

    public float read(String key) {
        return sharedPref.getFloat(key, 1);
    }

}
