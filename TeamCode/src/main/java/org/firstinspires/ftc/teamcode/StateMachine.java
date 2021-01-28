package org.firstinspires.ftc.teamcode;

import android.graphics.Bitmap;
import android.hardware.camera2.CameraAccessException;
import android.media.Image;
import android.provider.ContactsContract;
import android.util.Log;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcontroller.internal.FtcRobotControllerActivity;

import java.io.File;
import java.util.concurrent.TimeUnit;

import hortonvillerobotics.State;
import hortonvillerobotics.Timer;

import static org.firstinspires.ftc.teamcode.Robot.wheelSet1;
import static org.firstinspires.ftc.teamcode.Robot.wheelSet2;

class StateMachine{

    public String flag = "";
    //Robot rbt= Robot.getInstance();

    Robot rbt = Robot.getInstance();
    final String TAG = "StateMachine";
    final double wheelCircumference = 2.7222222222222222222222222222222 * Math.PI;
    final int countsPerRotation = 560;
    int current_number = 0;
    int state_in_progress = 1;
    int pos = 0;
    Timer t = new Timer();
    boolean timerOS = true;
    boolean initOS = true;
    boolean moveInit = true;

    long systemTime;

    @Override
    public String toString(){
        return "cn:"+current_number+"\tSIP:"+state_in_progress+"\tFlag:"+flag;
    }

    boolean next_state_to_execute(){
        return (state_in_progress == ++current_number);
    }
    void initializeMachine(){
        current_number = 0;
    }

    void incrementState(){
        state_in_progress++;
    }

    void reset(){
        state_in_progress = 1;
        current_number = 0;
    }

    public void runStates(State... states){
        for(State s : states) if(next_state_to_execute()) s.run();
    }

    boolean waitHasFinished(long milliseconds) {
        boolean returnVal = false;

        if (initOS) {
            systemTime = System.nanoTime() / 1000000;
            initOS = false;
        } else if ((System.nanoTime() / 1000000) - systemTime >= milliseconds) {
            initOS = true;
            returnVal = true;
        }

        return returnVal;
    }

    void SetFlag(StateMachine receiver, String key){
        if(next_state_to_execute()){
            receiver.flag = key;
            incrementState();
        }
    }

    void WaitForFlag(String key){
        if(next_state_to_execute()){
            if(flag.equals(key)){
                incrementState();
            }
        }
    }

    public void pause(long msec) {
        if(next_state_to_execute()) {
            if(timerOS){
                t.reset();
                timerOS = false;
            }
            if(t.hasTimeElapsed(msec)) {
                incrementState();
                timerOS = true;
            }
        }
    }

    void ClearFlag(){
        if(next_state_to_execute()){
            flag = "";
            incrementState();
        }
    }

    public void initRunToTarget(String motorName, int target, double power) {
        if (rbt.motors.get(motorName) != null) {
            rbt.setTarget(motorName, target);
            DcMotor temp = (DcMotor) rbt.motors.get(motorName);
            while(temp.getTargetPosition() == 0);
            try {
                TimeUnit.MILLISECONDS.sleep(10);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
            rbt.setPower(motorName, power);
        } else {
            Log.e(TAG, "initRunToTarget: failed to run motor: " + motorName + " to target: " + target + " at power: " + power);
        }
    }

    public void runToTarget(String motorName, int target, double power, boolean reset) {
        DcMotor motor = (DcMotor) rbt.motors.get(motorName);
        if(next_state_to_execute()) {
            if(moveInit && reset){
                rbt.resetEncoder(motorName);
                while(Math.abs(rbt.getEncoderCounts(motorName)) > 10);
                initRunToTarget(motorName, target, power);
//                rbt.setRunMode(motorName, DcMotor.RunMode.RUN_USING_ENCODER);
                moveInit = false;
            }else{
                initRunToTarget(motorName,target,power);
//                rbt.setRunMode(motorName, DcMotor.RunMode.RUN_USING_ENCODER);
                moveInit = false;
            }
//            initRunToTarget(motorName,target,power);
            while(rbt.getEncoderCounts(motorName)==null){
                try {
                    TimeUnit.MILLISECONDS.sleep(100);
                } catch (InterruptedException e) {
                    e.printStackTrace();
                }
            }
            if ((rbt.motors.get(motorName) != null) && !motor.isBusy()||(Math.abs(rbt.getEncoderCounts(motorName)) >= Math.abs(target)-20)) {
                rbt.setPower(motorName,0);
                if(reset) rbt.resetEncoder(motorName);
                rbt.setRunMode(motorName, DcMotor.RunMode.RUN_USING_ENCODER);
                moveInit = true;
                incrementState();
            }

        }
    }

    public void initRunToTarget(String motorName, int target, double power, boolean reset) {
        if (rbt.motors.get(motorName) != null) {
            if (reset) rbt.resetEncoder(motorName);
            initRunToTarget(motorName, target, power);
        }
    }

//    <<------------------------------ Mecanum Wheels --------------------------------->>


    void translate(double degrees, double power, double distance) { //Degrees0->Straight, Degrees 90 -> Left , Degrees -90 -> Right, Degrees 180 -> Backwards
        //boolean moveInit = true;
        if(next_state_to_execute()) {
            double wheelRotations = distance / wheelCircumference;
            int targetEncoderCounts = (int) (wheelRotations * countsPerRotation);
            double theta2 = (-45 - degrees) / 180.0 * Math.PI;
            int wheelSetEncoder1 = (int) (targetEncoderCounts * (Math.cos(theta2)));
            Log.d("ENCODERS", targetEncoderCounts + "");
            int wheelSetEncoder2 = (int) (-targetEncoderCounts * (Math.sin(theta2)));
            double wheelSetPower1 = power * (Math.cos(theta2));
            double wheelSetPower2 = power * -(Math.sin(theta2));

            Log.d("POWER1: ",wheelSetPower1+"");
            Log.d("POWER2: ",wheelSetPower2+"");
            //Log.d("ENC_TRANSLATE",String.format("MoveInit:%s",moveInit));
            //if(moveInit) {
            Log.d("ENCODERS", "(" + wheelSetEncoder1 + ":" + wheelSetPower1 + ") , (" + wheelSetEncoder2 + ":" + wheelSetPower2 + ")");

            if (moveInit) {
                rbt.initRunDriveToTarget(wheelSetEncoder1, wheelSetPower1, wheelSetEncoder2, wheelSetPower2, true);
                moveInit = false;
            }
            DcMotor temp = (DcMotor) rbt.motors.get(wheelSet1[0]);
            if ((!temp.isBusy()||(rbt.hasMotorEncoderReached(wheelSet1[0], wheelSetEncoder1) && (rbt.hasMotorEncoderReached(rbt.wheelSet2[1], wheelSetEncoder2))))) {
                Log.d("ENC_TRANSLATE", String.format("(%d,%d) / (%d,%d)",
                        rbt.getEncoderCounts(wheelSet1[0]),
                        rbt.getEncoderCounts(rbt.wheelSet2[1]),
                        wheelSetEncoder1, wheelSetEncoder2));

                rbt.setDrivePower(-wheelSetPower1,-wheelSetPower2);
                rbt.setPower(wheelSet1[0],0);
                rbt.setPower(wheelSet1[1],0);
                rbt.setPower(wheelSet2[0],0);
                rbt.setPower(wheelSet2[1],0);

                rbt.resetDriveEncoders();
                rbt.setDriveRunMode(DcMotor.RunMode.RUN_USING_ENCODER);
                moveInit = true;
                incrementState();
            }
        }
    }


    public void rotate(double degrees, double power) {
        if(next_state_to_execute()) {
            double turnCircumference = 16.5 * Math.PI; //changed but still have to test long-term rotate effect
            double wheelRotations = (turnCircumference / wheelCircumference) * (Math.abs(degrees) / 360);
            int targetEncoderCounts = (int) (wheelRotations * countsPerRotation);
            Log.d(TAG, "MOTOR turn: Target counts: " + targetEncoderCounts);

            initRunToTarget(rbt.wheelSetL[0], (int) (-Math.signum(degrees) * targetEncoderCounts), power);
            initRunToTarget(rbt.wheelSetL[1], (int) (-Math.signum(degrees) * targetEncoderCounts), power);
            initRunToTarget(rbt.wheelSetR[0], (int) (Math.signum(degrees) * targetEncoderCounts), power);
            initRunToTarget(rbt.wheelSetR[1], (int) (Math.signum(degrees) * targetEncoderCounts), power);

            Log.d(TAG, "BMOTOR Front Left "+rbt.getEncoderCounts("mtrFrontLeft"));
            Log.d(TAG, "BMOTOR Front Right "+rbt.getEncoderCounts("mtrFrontRight"));
            Log.d(TAG, "BMOTOR Back Left "+rbt.getEncoderCounts("mtrBackLeft"));
            Log.d(TAG, "BMOTOR Back Right "+rbt.getEncoderCounts("mtrBackRight"));

            if (rbt.hasMotorEncoderReached(rbt.wheelSetL[0], targetEncoderCounts)
                    && rbt.hasMotorEncoderReached(rbt.wheelSetR[0], targetEncoderCounts)) {
                Log.d(TAG,"turn: counts: "+rbt.getEncoderCounts(rbt.wheelSet1[0]));

                Log.d(TAG, "MOTOR Front Left "+rbt.getEncoderCounts("mtrFrontLeft"));
                Log.d(TAG, "MOTOR Front Right "+rbt.getEncoderCounts("mtrFrontRight"));
                Log.d(TAG, "MOTOR Back Left "+rbt.getEncoderCounts("mtrBackLeft"));
                Log.d(TAG, "MOTOR Back Right "+rbt.getEncoderCounts("mtrBackRight"));


                rbt.setPower(wheelSet1[0],0);
                rbt.setPower(wheelSet1[1],0);
                rbt.setPower(wheelSet2[0],0);
                rbt.setPower(wheelSet2[1],0);

                rbt.resetDriveEncoders();
                rbt.setDriveRunMode(DcMotor.RunMode.RUN_USING_ENCODER);
                incrementState();
            }
        }

    }

    public void setServoPosition(String servo, double pos){
        if(next_state_to_execute()){
            rbt.setServoPosition(servo,pos);
            incrementState();
        }
    }

    public void setServoPower(String servo, double power){
        if(next_state_to_execute()){
            rbt.setServoPower(servo,power);
            incrementState();
        }
    }

    public int getPos(){
        if(next_state_to_execute()) {
            return 1;

        }
        return 1;
    }


}