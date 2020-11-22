package org.firstinspires.ftc.teamcode;

public class TestRobotConfig extends RobotConfiguration{
    private double wheelCircumference = 4.4* Math.PI;
    private double turnDiameter = 16.8;
    private double countsPerRotation = 560;

    public double getWheelCircumference(){return wheelCircumference;}
    public double getTurnDiameter(){return turnDiameter;}
    public double getCountsPerRotation(){return countsPerRotation;}


    public static String[][] motors = {
            {"mtrBackRight", "forward"},
            {"mtrBackLeft", "forward"},
            {"mtrFrontLeft", "forward"},
            {"mtrFrontRight", "forward"}
    };

    private static String[][] servos = {

    };

    private static String[][] sensors = {
            {"IMU", ""}
    };

    public String[][] getMotors(){return motors;}
    public String[][] getServos(){return servos;}
    public String[][] getSensors(){return sensors;}

}
