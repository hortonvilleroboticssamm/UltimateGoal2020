package org.firstinspires.ftc.teamcode;
import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
    @Autonomous(name = "AutoBlueLeft", group = "Testing")
    public class AutoBlueLeft extends OpMode {
        Robot r;

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
//
//            if(r.getEncoderCounts("mtrFrontRight")>frontRightEncoder-3000){
//                r.setPower("mtrFrontRight",-0.4);
//                r.setPower("mtrFrontLeft",-0.5);
//                r.setPower("mtrBackRight",-0.4);
//                r.setPower("mtrBackLeft",-0.5);
//            }else{
//                r.setPower("mtrFrontRight",0);
//                r.setPower("mtrFrontLeft",0);
//                r.setPower("mtrBackRight",0);
//                r.setPower("mtrBackLeft",0);
//            }
//            telemetry.addData("LF Power",r.getPower("mtrFrontLeft"));
//            telemetry.addData("LF Power",r.getPower("mtrBackLeft"));
//            telemetry.addData("LF Power",r.getPower("mtrFrontRight"));
//            telemetry.addData("LF Power",r.getPower("mtrBackRight"));
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

      public void autonomousDrive(){
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

    public void scanRings(){







    }









}
