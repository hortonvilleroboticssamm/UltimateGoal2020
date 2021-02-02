package org.firstinspires.ftc.teamcode;


import android.os.Environment;

import com.google.gson.internal.bind.util.ISO8601Utils;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.json.JSONObject;

import java.io.File;
import java.io.FileInputStream;
import java.io.FileNotFoundException;
import java.io.FilenameFilter;
import java.io.InputStream;
import java.util.ArrayList;

import hortonvillerobotics.State;

@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name = "Autonomous", group = "Testing")
public class Autonomous1 extends OpMode {

    Robot r;
    StateMachine sm = new StateMachine();

    boolean gpPressed = false;
    //Angles
    double[] blue_y_p1 = new double[]{5,7,9};
    double[] blue_x_p1 = new double[]{-.5,.5,-.5};


    int pos = 0;

    boolean isPressed = false;

    String allianceColor = "blue"; //default

    long wait = 0; //default


    @Override
    public void init() {
        r = Robot.getInstance(this, new ParametersSelector());


    }

    @Override
    public void init_loop() {
        sm.runStates(() -> {
                    telemetry.addData("Alliance Color:", allianceColor);

                    if (gamepad1.x) {
                        allianceColor = "blue";
                    }
                    if (gamepad1.b) {
                        allianceColor = "red";
                    }
                    if (gamepad1.dpad_down) {
                        sm.incrementState();
                    }
                }, () -> {
                    telemetry.addData("Pos: ", pos);
                    if (gamepad1.x) {
                        pos = 1;
                    }
                    if (gamepad1.b) {
                        pos = 2;
                    }
                    if (gamepad1.dpad_down) {
                        sm.incrementState();
                    }
                }
        );

    }

    @Override
    public void start() {
        sm.reset();
    }

    @Override
    public void loop() {
        sm.initializeMachine();
        int wobblePos = sm.getPos();

        WobbleGoalGetter wobbleGoalGetter = new WobbleGoalGetter(r);

        telemetry.addData("Lower Senosr, Red: ", wobbleGoalGetter.getRed(true));
        //telemetry.addData("Lower Sensor, Blue: ", wobbleGoalGetter.getBlue(true));
        //telemetry.addData("Lower Sensor, Green: ", wobbleGoalGetter.getGreen(true));

        if(allianceColor.equals("blue")){


            switch(wobblePos){
                case 1:
                    sm.translate(0, r.safeSpeed, blue_y_p1[wobblePos-1]);
                    sm.translate(90, r.safeSpeed, blue_x_p1[wobblePos-1]);
                    break;
                case 2:
                    sm.translate(0, r.safeSpeed, blue_y_p1[wobblePos-1]);
                    sm.translate(-90, r.safeSpeed, blue_x_p1[wobblePos-1]);
                    break;
                case 3:
                    sm.translate(0, r.safeSpeed, blue_y_p1[wobblePos-1]);
                    sm.translate(-90, r.safeSpeed, blue_x_p1[wobblePos-1]);
                    break;
            }



        }

    }
}
