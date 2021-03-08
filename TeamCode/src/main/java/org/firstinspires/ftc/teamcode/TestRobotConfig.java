package org.firstinspires.ftc.teamcode;

public class TestRobotConfig extends RobotConfiguration{
    private double wheelCircumference = 4.4* Math.PI;
    private double turnDiameter = 16.8;
    private double countsPerRotation = 560;
    private double inchesPerEncoderCount = 0.02468;
    private double encoderCountsPerInch = 40.51864;

    public double getWheelCircumference(){return wheelCircumference;}
    public double getTurnDiameter(){return turnDiameter;}
    public double getCountsPerRotation(){return countsPerRotation;}
    //we need to go 65 inches for autonomous


    public static String[][] motors = {
            {"mtrBackRight", "Forward"},
            {"mtrBackLeft", "reverse"},
            {"mtrFrontLeft", "Forward"},
            {"mtrFrontRight", "forward"},
            {"mtrCollect1", "forward"},
            {"mtrCollect2","reverse"},
            {"mtrShoot", "forward"},
            {"mtrAngle","forward"}
    };

    private static String[][] servos = {
            {"srvArm","continuous"},
            {"srvClaw","continuous"}
    };

    private static String[][] sensors = {
            //{"IMU", "forward"}
            {"color0","color"}
    };

    public String[][] getMotors(){return motors;}
    public String[][] getServos(){return servos;}
    public String[][] getSensors(){return sensors;}

}
