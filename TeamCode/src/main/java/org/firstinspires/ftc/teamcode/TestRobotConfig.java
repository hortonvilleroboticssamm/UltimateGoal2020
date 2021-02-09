package org.firstinspires.ftc.teamcode;

public class TestRobotConfig extends RobotConfiguration{
    private double wheelCircumference = 4.4* Math.PI;
    private double turnDiameter = 16.8;
    private double countsPerRotation = 560;
    private double inchesPerEncoderCount = 0.02468;

    public double getWheelCircumference(){return wheelCircumference;}
    public double getTurnDiameter(){return turnDiameter;}
    public double getCountsPerRotation(){return countsPerRotation;}


    public static String[][] motors = {
            {"mtrBackRight", "reverse"},
            {"mtrBackLeft", "reverse"},
            {"mtrFrontLeft", "reverse"},
            {"mtrFrontRight", "forward"},
            {"mtrCollect", "forward"},
            {"mtrShoot", "forward"}
    };

    private static String[][] servos = {
            {"srvAngle", "continuous"}

    };

    private static String[][] sensors = {
            //{"IMU", "forward"}
    };

    public String[][] getMotors(){return motors;}
    public String[][] getServos(){return servos;}
    public String[][] getSensors(){return sensors;}

}
