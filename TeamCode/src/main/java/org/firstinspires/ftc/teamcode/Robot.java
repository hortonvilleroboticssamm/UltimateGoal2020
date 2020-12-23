package org.firstinspires.ftc.teamcode;

import android.graphics.Bitmap;
import android.util.Log;


import androidx.annotation.NonNull;
import androidx.annotation.Nullable;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import java.net.CookieHandler;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.concurrent.CopyOnWriteArrayList;
import java.util.concurrent.CountDownLatch;

import hortonvillerobotics.Timer;



//import android.support.annotation.Nullable;

public class Robot<T extends RobotConfiguration> {


    public static String TAG = "ROBOTCLASS";

    private static Robot currInstance;

    public Bitmap[] cameraSnapshots;
    double oldXValue=0;
    double oldYValue=0;
    double newXValue=0;
    double newYValue=0;
    String gc="True";

    public final String[] blockLocation = {"error"};

    BNO055 imu;
    Orientation angles;



    Telemetry telemetry;

    static String[] wheelSet1 = {"mtrFrontLeft", "mtrBackRight"};
    static String[] wheelSet2 = {"mtrFrontRight", "mtrBackLeft"};
    static String[] wheelSetL = {"mtrFrontLeft", "mtrBackLeft"};
    static String[] wheelSetR = {"mtrFrontRight", "mtrBackRight"};

    public static <T extends RobotConfiguration> Robot getInstance(OpMode opMode, T config) {
        currInstance = currInstance == null ? new Robot<T>(opMode, config) : currInstance;
        currInstance.opMode = opMode;
        currInstance.config = config;
        return currInstance;
    }

    public Map<String, DcMotor> motors;
    public Map<String, HardwareDevice> servos;
    public Map<String, HardwareDevice> sensors;

    List<String> flags = new CopyOnWriteArrayList<>();
    public OpMode opMode = null;
    public T config = null;

    public interface Task {
        void executeTasks();
    }

    public interface VelocityTask {
        double getSample();
    }

    private Robot(OpMode opMode, T config) {
        initialize(opMode, config);
    }

    public void initialize(@NonNull OpMode opMode, T config) {

        /*BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();*/


        //imu = opMode.hardwareMap.get(BNO055.class, "imu");
        

        motors = new HashMap<>();
        servos = new HashMap<>();
        sensors = new HashMap<>();
        cameraSnapshots = new Bitmap[3];
        flags.clear();

        for (String[] motorData : config.getMotors()) {
            try {
                String motorName = motorData[0];
                Log.i(TAG, "initialize: trying to add motor: " + motorName);
                DcMotor motor = (DcMotor) opMode.hardwareMap.get(motorName);

                motor.resetDeviceConfigurationForOpMode();
                motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

                if (motorData[1].equals("reverse")) {
                    motor.setDirection(DcMotorSimple.Direction.REVERSE);
                } else {
                    motor.setDirection(DcMotorSimple.Direction.FORWARD);
                }

                motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

                motors.put(motorName, motor);
            } catch (Exception e) {
                Log.e(TAG, "Failed to add motor: " + motorData[0]);
                e.printStackTrace();
            }
        }

        for (String[] servoData : config.getServos()) {
            try {
                String servoName = servoData[0];
                if (servoData.length > 1 && servoData[1].equalsIgnoreCase("continuous")) {
                    CRServo servo = (CRServo) opMode.hardwareMap.get(servoName);
                    servo.resetDeviceConfigurationForOpMode();
                    servos.put(servoName, servo);
                } else {
                    Servo servo = (Servo) opMode.hardwareMap.get(servoName);
                    servo.resetDeviceConfigurationForOpMode();
                    servos.put(servoName, servo);
                }
            } catch (Exception e) {
                Log.e(TAG, "Failed to add servo: " + servoData[0]);
                e.printStackTrace();
            }
        }

        for (String[] sensorData : config.getSensors()) {
            try {
                String sensorName = sensorData[0];
                if (sensorData.length > 1) {

                    HardwareDevice sensor = opMode.hardwareMap.get(sensorName);
                    sensor.resetDeviceConfigurationForOpMode();

                    if (sensor instanceof ColorSensor) {
                        Log.e("ROBOT", "SENSOR IS RIGHT TYPE");
                        ((ColorSensor) sensor).setI2cAddress(I2cAddr.create8bit(Integer.parseInt(sensorData[1], 16)));
                        ((ColorSensor) sensor).enableLed(true);
                    }

                    sensors.put(sensorName, sensor);
                }
            } catch (Exception e) {
                Log.e(TAG, "Failed to add sensor: " + sensorData[0]);
                e.printStackTrace();
            }
        }

        telemetry = opMode.telemetry;



    }



    /*public int getAngle(){

        return -1;
    }*/

    public void runParallel(@NonNull String endFlagName, @NonNull Task... tasks) {
        CountDownLatch l = new CountDownLatch(tasks.length);
        ArrayList<Thread> threads = new ArrayList<>();
        for (Task t : tasks) {
            threads.add(new Thread(() -> {
                try {
                    t.executeTasks();
                    l.countDown();
                }catch(Exception e){
                    e.printStackTrace();
                }
            }));
            threads.get(threads.size()-1).setName("Robot Multi-Thread Thread");
            threads.get(threads.size()-1).start();
        }
        new Thread(() -> {
            try {
                while(l.getCount() != 0){
                    if(opMode instanceof LinearOpMode && !((LinearOpMode) opMode).opModeIsActive() && ((LinearOpMode) opMode).isStopRequested())
                        for(Thread t : threads) t.join();
                }
                flags.add(endFlagName);
            } catch (InterruptedException ie) {
                ie.printStackTrace();
                Log.e(TAG, "runParallel: Failed to await latch");
            }
        }).start();

    }

    public void waitForFlag(@NonNull String flagName) {
        boolean flagFound = false;
        while (!flagFound && opModeIsActive()) {
            for (String s : flags)
                flagFound |= s.equals(flagName);
        }
        flags.remove(flagName);
        System.out.println("Flag \"" + flagName + "\" hit.");
    }


    //----ROBOT UTILITY FUNCTIONS----//
    public double[] gridCoordinates(){
        TestRobotConfig motors = new TestRobotConfig();
        //the first column is X values, the second column is Y values
        // the first row is Goal 1 (left most goal) second row is Goal 2 (right most goal)
        //max coordinate values are 5834.684 in both directions
        double[][] coordinates={
                {1154.781,5834.684},
                {4679.903,5834.684}
        };


//        while(gc=="True"){
//            for(int i=0;i<motors.getMotors().length;i++){
//                newXValue+=Math.cos(getAngle())*getEncoderCounts(motors.getMotors()[i][0]);
//                newYValue+=Math.sin(getAngle())*getEncoderCounts(motors.getMotors()[i][0]);
//            }
//            newXValue=newXValue/motors.getMotors().length;
//            newYValue=newYValue/motors.getMotors().length;
//            oldXValue=newXValue;
//            oldYValue=newYValue;
//
//        }
        return new double[]{newXValue, newYValue};
    }
    public void setPower(@NonNull String motorName, double power) {
        if (motors.get(motorName) != null) motors.get(motorName).setPower(power);
    }

    @Nullable
    public Integer getEncoderCounts(@NonNull String motorName) {
        return (motors.get(motorName) != null) ? motors.get(motorName).getCurrentPosition() : null;
    }

    @Nullable
    public Double getPower(@NonNull String motorName) {
        return (motors.get(motorName) != null) ? motors.get(motorName).getPower() : -.1;
    }

    public void resetEncoder(@NonNull String motorName) {
        if (motors.get(motorName) != null) {
            setRunMode(motorName, DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            setRunMode(motorName, DcMotor.RunMode.RUN_USING_ENCODER);
        } else {
            Log.e(TAG, "resetEncoder: motor is null: " + motorName);
        }
    }

    public void setRunMode(@NonNull String motorName, DcMotor.RunMode runMode) {
        if (motors.get(motorName) != null) motors.get(motorName).setMode(runMode);
    }

    public void setTarget(@NonNull String motorName, int target) {
        if (motors.get(motorName) != null) {
            setRunMode(motorName, DcMotor.RunMode.RUN_TO_POSITION);
            motors.get(motorName).setTargetPosition(target);
        }
    }

    public void initRunToTarget(@NonNull String motorName, int target, double power) {
        if (motors.get(motorName) != null) {
            setTarget(motorName, target);
            setPower(motorName, power);
        } else {
            Log.e(TAG, "initRunToTarget: failed to run motor: " + motorName + " to target: " + target + " at power: " + power);
        }
    }

    public void initRunToTarget(@NonNull String motorName, int target, double power, boolean reset) {
        if (motors.get(motorName) != null) {
            if (reset) resetEncoder(motorName);
            initRunToTarget(motorName, target, power);
        }
    }

    @Nullable
    public Integer getColorValue(String sensor, String channel) {
        if (sensors.get(sensor) != null && sensors.get(sensor) instanceof ColorSensor) {
            switch (channel) {
                case "red":
                    return ((ColorSensor) sensors.get(sensor)).red();
                case "blue":
                    return ((ColorSensor) sensors.get(sensor)).blue();
                case "green":
                    return ((ColorSensor) sensors.get(sensor)).green();
                case "alpha":
                    return ((ColorSensor) sensors.get(sensor)).alpha();
            }
        }
        return null;
    }



    public double getHeadingAngle(){ return 0.0d; }

    public void resetDriveEncoders() {
        resetEncoder("mtrLeftDrive");
        resetEncoder("mtrRightDrive");
    }

    public void setDrivePower(double lPow, double rPow) {
        setPower("mtrLeftDrive", lPow);
        setPower("mtrRightDrive", rPow);
    }

    public void setDriveEncoderTarget(int lTarget, int rTarget) {
        setTarget("mtrLeftDrive", lTarget);
        setTarget("mtrRightDrive", rTarget);
    }

    public void setDriveRunMode(DcMotor.RunMode rm) {
        setRunMode("mtrLeftDrive", rm);
        setRunMode("mtrRightDrive", rm);
    }

    public void initRunDriveToTarget(int lTarget, double lPow, int rTarget, double rPow) {
        initRunToTarget("mtrLeftDrive", lTarget, lPow);
        initRunToTarget("mtrRightDrive", rTarget, rPow);
    }

    public void initRunDriveToTarget(int lTarget, double lPow, int rTarget, double rPow, boolean reset) {
        if (reset) resetDriveEncoders();
        setDriveRunMode(DcMotor.RunMode.RUN_TO_POSITION);
        setDriveEncoderTarget(lTarget, rTarget);
        setDrivePower(lPow, rPow);
    }

    public boolean opModeIsActive() {
        return opMode instanceof LinearOpMode && ((LinearOpMode) opMode).opModeIsActive();
    }

    public double calculateVelocity(VelocityTask t, long sampleTimeMS) {
        Timer timer = new Timer();
        double startVal = t.getSample();
        while (!timer.hasTimeElapsed(sampleTimeMS)) ;
        double currentVelocity = (t.getSample() - startVal) / sampleTimeMS * 1000.;
        Log.d(TAG, "calculateVelocity: current velocity: " + currentVelocity);
        return currentVelocity;
    }

    public void setServoPosition(@NonNull String servoName, double position) {
        HardwareDevice servo = servos.get(servoName);
        if (servo != null) {
            if (servo instanceof Servo) ((Servo) servo).setPosition(position);
            else if (servo instanceof CRServo) setServoPower(servoName, position);
        } else Log.e(TAG, "setServoPosition: servo is null: " + servoName);
    }

    public void setServoPower(@NonNull String servoName, double power) {
        HardwareDevice servo = servos.get(servoName);
        if (servo != null) {
            if (servo instanceof CRServo) ((CRServo) servo).setPower(power);
            else if (servo instanceof Servo) setServoPosition(servoName, power);

        } else Log.e(TAG, "setServoPower: CR servo is null: " + servoName);
    }

    @Nullable
    public Boolean hasMotorEncoderReached(@NonNull String motorName, int encoderCount) {
        return (motors.get(motorName) != null) ? Math.abs(getEncoderCounts(motorName)) >= Math.abs(encoderCount) : null;
    }

    //----ROBOT FUNCTIONS BEGIN----//
    //----ROBOT FUNCTIONS BEGIN----//

    public void pause(long msec) {
        Timer t = new Timer();
        while (opModeIsActive() && !t.hasTimeElapsed(msec)) ;
    }

    //DRIVE WITH DEFAULT POWER OF 72%
    public void drive(double distance) {
        drive(distance, 0.72);
    }

    //DRIVE WITH SPECIFIED POWER
    public void drive(double distance, double power) {
        DcMotor mtrLeftDrive = motors.get("mtrLeftDrive"), mtrRightDrive = motors.get("mtrRightDrive");
        double wheelRotations = distance / config.getWheelCircumference();
        int targetEncoderCounts = (int) (wheelRotations * config.getCountsPerRotation());
        Log.i(TAG, "drive: Target counts: " + targetEncoderCounts);

        initRunDriveToTarget(-targetEncoderCounts, power, -targetEncoderCounts, power, true);

        while (opModeIsActive()) {

            Log.d(TAG, "turn: current right count: " + mtrRightDrive.getCurrentPosition());
            Log.d(TAG, "turn: current left count: " + mtrLeftDrive.getCurrentPosition());

            opMode.telemetry.addData("target", targetEncoderCounts);
            opMode.telemetry.update();

            if (Math.abs(getEncoderCounts("mtrLeftDrive")) >= Math.abs(targetEncoderCounts) - 20) {
                setPower("mtrLeftDrive", 0);
            } else {
                setPower("mtrLeftDrive", power);
            }

            if (Math.abs(getEncoderCounts("mtrRightDrive")) >= Math.abs(targetEncoderCounts) - 20) {
                setPower("mtrRightDrive", 0);
            } else {
                setPower("mtrRightDrive", power);
            }

            if (getPower("mtrLeftDrive") == 0 && getPower("mtrRightDrive") == 0) break;
        }


        resetDriveEncoders();
        Log.v(TAG, "drive: Successfully drove to target of " + distance + " inches");

    }


    //TURN WITH DEFAULT POWER OF 40%
    public void turn(double degrees) {
        turn(degrees, 0.4);
    }

    //TURN WITH SPECIFIED POWER
    public void turn(double degrees, double power) {
        DcMotor mtrLeftDrive = motors.get("mtrLeftDrive"), mtrRightDrive = motors.get("mtrRightDrive");
        double turnCircumference = config.getTurnDiameter() * Math.PI;
        double wheelRotations = (turnCircumference / config.getWheelCircumference()) * (Math.abs(degrees) / 360);
        int targetEncoderCounts = (int) (wheelRotations * config.getCountsPerRotation());
        Log.i(TAG, "turn: Target counts: " + targetEncoderCounts);
        setDriveRunMode(DcMotor.RunMode.RUN_TO_POSITION);


        if (degrees > 0) {

            initRunDriveToTarget(targetEncoderCounts, power, -targetEncoderCounts, power, true);

        } else {

            initRunDriveToTarget(-targetEncoderCounts, power, targetEncoderCounts, power, true);

        }

        while (opModeIsActive()) {

            Log.d(TAG, "turn: current right count: " + mtrRightDrive.getCurrentPosition());
            Log.d(TAG, "turn: current left count: " + mtrLeftDrive.getCurrentPosition());

            if (Math.abs(getEncoderCounts("mtrLeftDrive")) >= Math.abs(targetEncoderCounts) - 20) {
                setPower("mtrLeftDrive", 0);
            } else {
                setPower("mtrLeftDrive", power);
            }

            if (Math.abs(getEncoderCounts("mtrRightDrive")) >= Math.abs(targetEncoderCounts) - 20) {
                setPower("mtrRightDrive", 0);
            } else {
                setPower("mtrRightDrive", power);
            }

            if (getPower("mtrLeftDrive") == 0 && getPower("mtrRightDrive") == 0) break;
        }

        mtrLeftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        mtrRightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Log.v(TAG, "turn: Successfully turned to target of " + degrees + " degrees");
    }

    /* DEGREES CONTROLS THE DISTANCE OF THE TURN
     * THE SIGN OF DEGREES CONTROLS WHICH MOTOR
     * THE SIGN OF POWER CONTROLS THE DIRECTION OF THE MOTOR */
    public void owTurn(double degrees, double power) {
        double turnCircumference = 2 * config.getTurnDiameter() * Math.PI;
        double wheelRotations = (turnCircumference / config.getWheelCircumference()) * (Math.abs(degrees) / 360);
        int targetEncoderCounts = (int) (wheelRotations * config.getCountsPerRotation() * Math.signum(power));

        Log.i(TAG, "owturn: Target counts: " + targetEncoderCounts);

        //TODO TEST REMOVING THE FOLLOWING LINE OF CODE BECAUSE IT IS UNNECCESSARY AND POSSIBLY GIVING ERRORS
//        setDriveRunMode(DcMotor.RunMode.RUN_TO_POSITION);

        boolean targetReached = false;

        if (degrees > 0) {

            if (power < 0) {
                Log.i(TAG, "owturn: power less than 0");
            }

            initRunDriveToTarget(0, 0, targetEncoderCounts, power, true);

            while (opModeIsActive() && Math.abs(getEncoderCounts("mtrRightDrive")) < Math.abs(targetEncoderCounts) - 20) {
                Log.d(TAG, "owturn: current right count: " + getEncoderCounts("mtrRightDrive"));
                Log.d(TAG, "owturn: current right power: " + getPower("mtrRightDrive"));
            }
            setDrivePower(0.0, 0.0);

        } else {

            if (power < 0) {
                Log.i(TAG, "owturn: power less than 0");
            }

            initRunDriveToTarget(targetEncoderCounts, power, 0, 0, true);

            while (opModeIsActive() && Math.abs(getEncoderCounts("mtrLeftDrive")) < Math.abs(targetEncoderCounts) - 20) {
                Log.d(TAG, "owturn: current left count: " + getEncoderCounts("mtrLeftDrive"));
                Log.d(TAG, "owturn: current left power: " + getPower("mtrLeftDrive"));
            }
            setDrivePower(0.0, 0.0);

        }
    }

    //METHOD TO BE CALLED UPON COMPLETION OF AN AUTONOMOUS PROGRAM IF ENCODERS ARE DESIRED TO BE RESET
    public void finish() {
        Log.i(TAG, "finish: entering finish phase");
        for (String m : motors.keySet()) resetEncoder(m);
    }

    //SEASON SPECIFIC FUNCTIONS
    //SEASON SPECIFIC FUNCTIONS

    //TODO implement this method


    public void runToTarget(@NonNull String mtr, int encoder, double speed) {
        initRunToTarget(mtr, encoder, speed, true);
        while (!hasMotorEncoderReached(mtr, encoder)) ;
        setRunMode(mtr, DcMotor.RunMode.RUN_USING_ENCODER);
        setPower(mtr, 0);
    }


}
