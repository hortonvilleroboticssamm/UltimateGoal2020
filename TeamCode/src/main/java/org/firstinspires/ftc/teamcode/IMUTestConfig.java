package org.firstinspires.ftc.teamcode;

public class IMUTestConfig extends RobotConfiguration{
    private double wheelCircumference = 4.4* Math.PI;
    private double turnDiameter = 16.8;
    private double countsPerRotation = 560;

    public double getWheelCircumference(){return wheelCircumference;}
    public double getTurnDiameter(){return turnDiameter;}
    public double getCountsPerRotation(){return countsPerRotation;}


    public static String[][] motors = {

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
