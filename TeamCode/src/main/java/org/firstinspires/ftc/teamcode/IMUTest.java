package org.firstinspires.ftc.teamcode;
// Cade Help!!
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import java.util.ArrayList;

import hortonvillerobotics.FinalRobotConfiguration;

@TeleOp(name = "IMUTest", group = "TeleOp")
public class IMUTest extends OpMode{

    Robot r;

    IMUTestConfig imuTestConfig = new IMUTestConfig();

    @Override
    public void init() {
        r = Robot.getInstance(this, imuTestConfig);
        r.initialize(this, new TestRobotConfig());
        //r.setDriveRunMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    @Override
    public void loop() {
      //  r.getAngle();
    }
}
