package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import hortonvillerobotics.FinalRobotConfiguration;

@TeleOp(name = "TeleOPTest2020", group = "TeleOp")
public class TeleOP extends OpMode {
    Robot r;

    double drivePowerScale = 1;


    double theta1 = 0;


    @Override
    public void init() {
        r = Robot.getInstance(this, new TestRobotConfig());
        r.initialize(this, new TestRobotConfig());
        r.setDriveRunMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        telemetry.addData("Init Success", 69);
    }

    @Override
    public void loop() {




        //CONTROLLER 1
        //testxcvzv
            /*
            Tank Drive Controls
            Configured for Left-Handed Drivers


            Left Stick      -       Left Power
            Right Stick     -       Right Power
            L. Stick Button -       Cut Left Power
            R. Stick Button -       Cut Right Power
            Left Trigger    -       25% Drive Power
            Left Bumper     -       50% Drive Power
             */

        telemetry.addData("Doing it: ", 69);
        if (Math.abs(gamepad1.right_stick_x) < 0.075) {
            double x = gamepad1.left_stick_x;
            double y = -gamepad1.left_stick_y;

            theta1 = ((Math.atan(y / x)));
            //This series of if statements prevents us from dividing by 0
            //Because we divide by X, X != 0
            if (x == 0 && y > 0) {
                theta1 = Math.PI / 2;
            } else if (x == 0 && y < 0) {
                theta1 = 3 * Math.PI / 2;
            } else if (x < 0) {
                theta1 = Math.atan(y / x) + Math.PI;
            }
            double theta2 = Math.PI / 4 - theta1+Math.PI;
            double hyp = Math.sqrt(x * x + y * y);
            boolean motorBand = Math.abs(x) > .05 || Math.abs(y)> .05;
            telemetry.addData("MotorBand", motorBand);
            telemetry.addData("x", x);
            telemetry.addData("y", y);
            double speedControl = gamepad1.right_bumper ? 1 : gamepad1.right_trigger > .4 ? .25 : .5;
            r.setPower(Robot.wheelSet1[0], motorBand ?  hyp * Math.cos(theta2) * speedControl : 0);
            r.setPower(Robot.wheelSet2[0], motorBand ? -hyp * Math.sin(theta2) * speedControl : 0);
            r.setPower(Robot.wheelSet1[1], motorBand ?  hyp * Math.cos(theta2) * speedControl : 0);
            r.setPower(Robot.wheelSet2[1], motorBand ? -hyp * Math.sin(theta2) * speedControl : 0);
            telemetry.addData("Power", r.getPower("mtrBackLeft"));
            telemetry.addData("hyp", hyp);
            telemetry.addData("theta2", theta2);
            telemetry.addData("SpeedControl", speedControl);
        }else {
            r.setPower(Robot.wheelSetL[0], gamepad1.right_stick_x/2);
            r.setPower(Robot.wheelSetL[1], gamepad1.right_stick_x/2);
            r.setPower(Robot.wheelSetR[0], -gamepad1.right_stick_x/2);
            r.setPower(Robot.wheelSetR[1], -gamepad1.right_stick_x/2);
        }


        drivePowerScale = gamepad1.left_trigger >= .5 ? 0.25 : gamepad1.left_bumper ? 0.7 : 0.4;
        telemetry.addData("drivePowerScale: ", drivePowerScale);
        r.setDrivePower(Math.abs(gamepad1.left_stick_y) > 0.05 && !gamepad1.left_stick_button ? drivePowerScale * gamepad1.left_stick_y : 0, Math.abs(gamepad1.right_stick_y) > 0.05 && !gamepad1.right_stick_button ? drivePowerScale * gamepad1.right_stick_y : 0);

    }
}
