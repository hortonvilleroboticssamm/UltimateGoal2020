package org.firstinspires.ftc.teamcode;

import android.util.Log;
import android.widget.Toast;

import com.qualcomm.robotcore.hardware.ColorSensor;

public class WobbleGoalGetter {

    final String LOWER_SENSOR_NAME = "3c";
    final String HIGHER_SENSOR_NAME = "higherSensor";

    int lowerGreenBand = 200;
    int upperGreenBand = 255;

    int lowerRedBand = 200;
    int upperRedBand = 255;

    Robot r;

    public WobbleGoalGetter(Robot r){
        this.r = r;
    }

    public int getPos(){
        if(getGreen(true) > lowerGreenBand && getGreen(true) < upperGreenBand &&
            getRed(true) > lowerRedBand && getRed(true) < upperRedBand) {
            if(getGreen(false) > lowerGreenBand && getGreen(false) < upperGreenBand &&
                getRed(false) > lowerRedBand && getRed(false) < upperRedBand){
                return 3;
            } else {
                return 2;
            }

        }else
            return 1;
    }

    public int getRed(boolean isLower){
        if(r.sensors == null){
            return -2;
        }
        try {
            if (isLower) {
                return r.getColorValue(LOWER_SENSOR_NAME, "red");
            } else {
                return r.getColorValue(HIGHER_SENSOR_NAME, "red");
            }
        } catch(Exception e){
            Log.v("getRed()", e+"");
        }
        return -1;
    }

    public int getBlue(boolean isLower){
        if(isLower){
            return r.getColorValue(LOWER_SENSOR_NAME, "blue");
        } else {
            return r.getColorValue(HIGHER_SENSOR_NAME, "blue");
        }
    }

    public int getGreen(boolean isLower){
        if(isLower){
            return r.getColorValue(LOWER_SENSOR_NAME, "green");
        } else {
            return r.getColorValue(HIGHER_SENSOR_NAME, "green");
        }
    }

}
