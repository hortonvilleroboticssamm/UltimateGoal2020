package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import hortonvillerobotics.Robot;
import hortonvillerobotics.RobotConfiguration;

public class TeleOP extends OpMode {
    Robot r;

    double drivePowerScale = 1;


    @Override
    public void init() {
        r = Robot.getInstance(this, new RobotConfiguration());
        r.initialize(this, new RobotConfiguration());
        r.setDriveRunMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
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

        drivePowerScale = gamepad1.left_trigger >= .5 ? 0.25 : gamepad1.left_bumper ? 0.7 : 0.4;

        r.setDrivePower(Math.abs(gamepad1.left_stick_y) > 0.05 && !gamepad1.left_stick_button ? drivePowerScale * gamepad1.left_stick_y : 0, Math.abs(gamepad1.right_stick_y) > 0.05 && !gamepad1.right_stick_button ? drivePowerScale * gamepad1.right_stick_y : 0);

    }
}
