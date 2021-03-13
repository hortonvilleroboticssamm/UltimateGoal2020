package org.firstinspires.ftc.teamcode;


import android.os.Environment;

import com.google.gson.internal.bind.util.ISO8601Utils;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.json.JSONObject;

import java.io.File;
import java.io.FileInputStream;
import java.io.FileNotFoundException;
import java.io.FilenameFilter;
import java.io.InputStream;
import java.util.ArrayList;
import java.util.function.BiConsumer;

import hortonvillerobotics.State;

@Autonomous(name = "AutoBlueRight", group = "Testing")
public class AutoBlueRight extends OpMode {
    Robot r;
    //StateMachine sm = new StateMachine();

    /*boolean gpPressed = false;
    //Angles
    double[] blue_y_p1 = new double[]{5, 7, 9};
    double[] blue_x_p1 = new double[]{-.5, .5, -.5};


    int pos = 0;

    boolean isPressed = false;

    String allianceColor = "blue"; //default

    long wait = 0; //default


    @Override
    public void init() {
        r = Robot.getInstance(this, new ParametersSelector());
   pos = 2;
                        }
                        if (gamepad1.dpad_down) {
                            sm.incrementState();
                        }
                    }
            )
*/

    // }

//        @Override
//        public void init_loop() {
//
//
//            sm.runStates(() -> {
//                        telemetry.addData("Alliance Color:", allianceColor);
//
//                        if (gamepad1.x) {
//                            allianceColor = "blue";
//                        }
//                        if (gamepad1.b) {
//                            allianceColor = "red";
//                        }
//                        if (gamepad1.dpad_down) {
//                            sm.incrementState();
//                        }
//                    }, () -> {
//                        telemetry.addData("Pos: ", pos);
//                        if (gamepad1.x) {
//                            pos = 1;
//                        }
//                        if (gamepad1.b) {
//
//        }

//    @Override
//}n

    // Cade Help!
    double frontRightEncoder;

    public void init() {
        r = Robot.getInstance(this, new TestRobotConfig());
        r.initialize(this, new TestRobotConfig());
        frontRightEncoder = r.getEncoderCounts("mtrFrontRight");
    }

    public void loop() {
        toRings();
        ColorSensor color0;
        ColorSensor color1;
        color0=hardwareMap.get(ColorSensor.class,"color0");
        color1=hardwareMap.get(ColorSensor.class,"color1");

        if (color1.red()>240 && color1.green() >100){
            positionBlueC();
        }else if(color0.red()>240 && color0.green() >100){
            positionBlueB();
        }else{
            positionBlueA();
        }
//        if(r.getEncoderCounts("mtrFrontRight")>frontRightEncoder-3000){
//            r.setPower("mtrFrontRight",-0.4);
//            r.setPower("mtrFrontLeft",-0.5);
//            r.setPower("mtrBackRight",-0.4);
//            r.setPower("mtrBackLeft",-0.5);
//        }else{
//            r.setPower("mtrFrontRight",0);
//            r.setPower("mtrFrontLeft",0);
//            r.setPower("mtrBackRight",0);
//            r.setPower("mtrBackLeft",0);
//        }
//        telemetry.addData("LF Power",r.getPower("mtrFrontLeft"));
//        telemetry.addData("LF Power",r.getPower("mtrBackLeft"));
//        telemetry.addData("LF Power",r.getPower("mtrFrontRight"));
//        telemetry.addData("LF Power",r.getPower("mtrBackRight"));
        //       r.setPower("mtrShoot", -1.0);


    }
    public void toRings(){
        double forwardEncoder=r.getEncoderCounts("mtrFrontRight");
        if(r.getEncoderCounts("mtrFrontRight")<=forwardEncoder+r.setEncoder(30)){
            r.setPower("mtrFrontRight", 0.5);
            r.setPower("mtrFrontLeft", 0.5);
            r.setPower("mtrBackRight", 0.5);
            r.setPower("mtrBackLeft", 0.5);
        }else{
            r.setPower("mtrFrontRight", 0.0);
            r.setPower("mtrFrontLeft", 0.0);
            r.setPower("mtrBackRight", 0.0);
            r.setPower("mtrBackLeft", 0.0);
        }
        double sideEncoder=r.getEncoderCounts("mtrFrontRight");
        if(r.getEncoderCounts("mtrFrontRight")<=sideEncoder+r.setEncoder(12)){
            r.setPower("mtrFrontRight", -0.5);
            r.setPower("mtrFrontLeft", 0.5);
            r.setPower("mtrBackRight", 0.5);
            r.setPower("mtrBackLeft", -0.5);
        }else{
            r.setPower("mtrFrontRight", 0.0);
            r.setPower("mtrFrontLeft", 0.0);
            r.setPower("mtrBackRight", 0.0);
            r.setPower("mtrBackLeft", 0.0);
        }
    }
    public void positionBlueA(){
        double forwardEncoder=r.getEncoderCounts("mtrFrontRight");
        if(r.getEncoderCounts("mtrFrontRight")<=forwardEncoder+r.setEncoder(45)){
            r.setPower("mtrFrontRight", 0.5);
            r.setPower("mtrFrontLeft", 0.5);
            r.setPower("mtrBackRight", 0.5);
            r.setPower("mtrBackLeft", 0.5);
        }else{
            r.setPower("mtrFrontRight", 0.0);
            r.setPower("mtrFrontLeft", 0.0);
            r.setPower("mtrBackRight", 0.0);
            r.setPower("mtrBackLeft", 0.0);
        }
        double sideEncoder=r.getEncoderCounts("mtrFrontRight");
        if(r.getEncoderCounts("mtrFrontRight")<=sideEncoder+r.setEncoder(24)){
            r.setPower("mtrFrontRight", 0.5);
            r.setPower("mtrFrontLeft", -0.5);
            r.setPower("mtrBackRight", -0.5);
            r.setPower("mtrBackLeft", 0.5);
        }else{
            r.setPower("mtrFrontRight", 0.0);
            r.setPower("mtrFrontLeft", 0.0);
            r.setPower("mtrBackRight", 0.0);
            r.setPower("mtrBackLeft", 0.0);
        }
    }
    public void positionBlueB(){
        double forwardEncoder=r.getEncoderCounts("mtrFrontRight");
        if(r.getEncoderCounts("mtrFrontRight")<=forwardEncoder+r.setEncoder(69)){
            r.setPower("mtrFrontRight", 0.5);
            r.setPower("mtrFrontLeft", 0.5);
            r.setPower("mtrBackRight", 0.5);
            r.setPower("mtrBackLeft", 0.5);
        }else{
            r.setPower("mtrFrontRight", 0.0);
            r.setPower("mtrFrontLeft", 0.0);
            r.setPower("mtrBackRight", 0.0);
            r.setPower("mtrBackLeft", 0.0);
        }
    }
    public void positionBlueC(){
        double forwardEncoder=r.getEncoderCounts("mtrFrontRight");
        if(r.getEncoderCounts("mtrFrontRight")<=forwardEncoder+r.setEncoder(91)){
            r.setPower("mtrFrontRight", 0.5);
            r.setPower("mtrFrontLeft", 0.5);
            r.setPower("mtrBackRight", 0.5);
            r.setPower("mtrBackLeft", 0.5);
        }else{
            r.setPower("mtrFrontRight", 0.0);
            r.setPower("mtrFrontLeft", 0.0);
            r.setPower("mtrBackRight", 0.0);
            r.setPower("mtrBackLeft", 0.0);
        }
        double sideEncoder=r.getEncoderCounts("mtrFrontRight");
        if(r.getEncoderCounts("mtrFrontRight")<=sideEncoder+r.setEncoder(24)){
            r.setPower("mtrFrontRight", 0.5);
            r.setPower("mtrFrontLeft", -0.5);
            r.setPower("mtrBackRight", -0.5);
            r.setPower("mtrBackLeft", 0.5);
        }else{
            r.setPower("mtrFrontRight", 0.0);
            r.setPower("mtrFrontLeft", 0.0);
            r.setPower("mtrBackRight", 0.0);
            r.setPower("mtrBackLeft", 0.0);
        }
    }

}

//stop();


//        @Override
//        public void stop(){
//    }
//Cade's Original Autonomous
 /*   public void autonomousDrive(){
            frontRightEncoder=r.getEncoderCounts("mtrFrontRight");
        while(r.getEncoderCounts("mtrFrontRight")<frontRightEncoder+2634) {
            r.setPower("mtrFrontRight", 0.5);
            r.setPower("mtrFrontLeft", 0.5);
            r.setPower("mtrBackRight", 0.5);
            r.setPower("mtrBackLeft", 0.5);
        }
        r.setPower("mtrFrontRight",1.0);
        r.setPower("mtrFrontLeft",1.0);
        r.setPower("mtrBackRight",1.0);
        r.setPower("mtrBackLeft",1.0);
        r.setPower("mtrShoot",1.0);
    }
*/
        /*WobbleGoalGetter wobbleGoalGetter = new WobbleGoalGetter(r);

        telemetry.addData("Lower Senosr, Red: ", wobbleGoalGetter.getRed(true));
        //telemetry.addData("Lower Sensor, Blue: ", wobbleGoalGetter.getBlue(true));
        //telemetry.addData("Lower Sensor, Green: ", wobbleGoalGetter.getGreen(true));

        if(allianceColor.equals("blue")){


  /*          switch(wobblePos){
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

*/

// }

//package org.firstinspires.ftc.teamcode;
//
//public class AutoBlueRight {
//}
