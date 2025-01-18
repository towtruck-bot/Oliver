package org.firstinspires.ftc.teamcode.subsystems.deposit;

import android.util.Log;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.utils.Globals;
import org.firstinspires.ftc.teamcode.utils.TelemetryUtil;
import org.firstinspires.ftc.teamcode.utils.Utils;

@Config
public class Deposit {
    public enum State{
        IDLE,
        TRANSFER_PREPARE,
        TRANSFER_WAIT,
        TRANSFER_GRAB,
        TRANSFER_CLOSE,
        TRANSFER_FINISH,
        HOLD,
        SAMPLE_RAISE,
        SAMPLE_WAIT,
        SAMPLE_DEPOSIT,
        OUTTAKE_MOVE,
        OUTTAKE_RELEASE,
        GRAB_MOVE,
        GRAB_WAIT,
        GRAB,
        GRAB_RETRACT,
        GRAB_HOLD,
        SPECI_RAISE,
        SPECI_DEPOSIT,
        RELEASE,
        RETRACT
    }

    public State state;

    private final Robot robot;
    public final Slides slides;
    public final Arm arm;

    /// x values are measured from the base of the arm
    //// y values are measured from the ground
    //// intake is positive x direction

    //private final double baseHeight = 10.75;

    // y is measured from slides 0

    // prepare for transfer positions
    public static double intakeWaitRad = 0.35, intakeWaitClawRad = -1.9, intakeWaitY = 0.0;
    // transfer positions, move in to grab
    public static double intakeRad = -0.1, intakeY = 0.0, intakeClawRad = -1.9;
    // moving positions with a sample
    public static double holdRad = 0.0, holdY = 0.0, holdClawRad = 2;
    // sample basket positions
    public static double sampleLY = 16.75, sampleHY = 33.85, sampleRad = 2.4, sampleClawRad = 0.6;
    // outtake positions, drop behind robot
    public static double outtakeRad = Math.PI, outtakeY = 0.0, outtakeReleaseRad = 0.3;
    // grabbing positions, holdGrab -> off the wall, grabRetract --> moving with a specimen
    public static double holdGrabRad = 0.3, grabRetractRad = 2.0;
    // specimen chamber positions
    public static double speciLRad = 2.75, speciLClawRad = 0.0, speciLSY = 0.0, speciLEY = 0.0;
    public static double  speciHRad = 1.8, speciHClawRad = 0.05, speciHY = 8.0, speciHEY = 18.0;

    private boolean high = true, auto = false;

    public Deposit(Robot robot){
        this.robot = robot;
        this.slides = new Slides(this.robot);

        arm = new Arm(this.robot);

        state = Globals.hasSpecimenPreload ? State.HOLD : State.RETRACT;
    }

    public void update(){
        switch(state){
            case IDLE:
                break;
            case TRANSFER_PREPARE:
                moveToWithRad(intakeWaitRad, intakeWaitY);
                arm.setClawRotation(intakeWaitClawRad, 1.0);
                arm.speciOpen();

                if(arm.inPosition() && slides.inPosition(0.8) && arm.clawFinished()){
                    state = State.TRANSFER_WAIT;
                }
                break;
            case TRANSFER_WAIT:
                moveToWithRad(intakeWaitRad, intakeWaitY);
                arm.setClawRotation(intakeWaitClawRad, 1.0);
                break;
            case TRANSFER_GRAB:
                moveToWithRad(intakeRad, intakeY);
                arm.setClawRotation(intakeClawRad, 1.0);

                if(arm.inPosition() && slides.inPosition(1)){ // Need to figure out this threshold better
                    state = State.TRANSFER_CLOSE;
                }
                break;
            case TRANSFER_CLOSE:
                moveToWithRad(intakeRad, intakeY);
                arm.setClawRotation(intakeClawRad, 1.0);

                arm.speciClose();

                if(arm.clawFinished()){
                    robot.clawIntake.release();
                    state = State.TRANSFER_FINISH;
                }
                break;
            case TRANSFER_FINISH:
                moveToWithRad(holdRad, holdY);
                arm.clawRotation.setTargetAngle(holdClawRad, 1.0);

                if(arm.inPosition()){
                    state = State.HOLD;
                }
                break;
            case HOLD:
                moveToWithRad(holdRad, holdY);
                arm.clawRotation.setTargetAngle(holdClawRad, 1.0);

                break;
            case SAMPLE_RAISE:
                moveToWithRad(sampleRad, targetY);
                arm.setClawRotation(sampleClawRad, 1.0);

                if(arm.inPosition() && slides.inPosition(0.8)){
                    state = State.SAMPLE_WAIT;
                }
                break;
            case SAMPLE_WAIT:
                moveToWithRad(sampleRad, targetY);
                arm.setClawRotation(sampleClawRad, 1.0);
                break;
            case SAMPLE_DEPOSIT:
                moveToWithRad(sampleRad, targetY);
                arm.setClawRotation(sampleClawRad, 1.0);
                arm.sampleOpen();

                if(arm.clawFinished()){
                    state = State.RETRACT;
                }
                break;
            case OUTTAKE_MOVE:
                moveToWithRad(outtakeRad, outtakeY);
                arm.clawRotation.setTargetAngle(outtakeReleaseRad, 1.0);

                if(arm.inPosition() && slides.inPosition(0.8)){
                    state = State.OUTTAKE_RELEASE;
                }
                break;
            case OUTTAKE_RELEASE:
                moveToWithRad(outtakeRad, outtakeY);
                arm.clawRotation.setTargetAngle(outtakeReleaseRad, 1.0);

                arm.sampleOpen();

                if(arm.clawFinished()){
                    state = State.RETRACT;
                }
                break;
            case GRAB_MOVE:
                moveToWithRad(holdRad, holdY);
                arm.setClawRotation(holdGrabRad, 1.0);
                arm.speciOpen();

                if(arm.inPosition() && slides.inPosition(0.8) && arm.clawFinished()){
                    state = State.GRAB_WAIT;
                }
                break;
            case GRAB_WAIT:
                moveToWithRad(holdRad, holdY);
                arm.setClawRotation(holdGrabRad, 1.0);
                break;
            case GRAB:
                moveToWithRad(holdRad, holdY);
                arm.setClawRotation(holdGrabRad, 1.0);
                arm.speciClose();

                if(arm.clawFinished()){
                    state = State.GRAB_RETRACT;
                }
                break;
            case GRAB_RETRACT:
                moveToWithRad(grabRetractRad, holdY);
                arm.setClawRotation(speciHClawRad, 1.0);

                if(arm.inPosition()){
                    state = State.GRAB_HOLD;
                }
                break;
            case GRAB_HOLD:
                moveToWithRad(grabRetractRad, holdY);
                arm.setClawRotation(speciHClawRad, 1.0);
                break;
            case SPECI_RAISE:
                moveToWithRad(speciHRad, targetY);
                arm.setClawRotation(speciHClawRad, 1.0);

                if(auto && arm.inPosition() && arm.clawFinished()){
                    state = State.SPECI_DEPOSIT;
                }
                break;
            case SPECI_DEPOSIT:
                moveToWithRad(speciHRad, targetY); //TODO: In AUTO, this will have to be set to corresponding height, left here as placeholder. We may never need it either, who knows
                arm.setClawRotation(speciHClawRad, 1.0);
                break;
            case RELEASE:
                arm.speciOpen();

                if(arm.clawFinished()){
                    state = State.RETRACT;
                }
                break;
            case RETRACT:
                moveToStart();

                if(arm.inPosition() && slides.inPosition(0.8)){
                    state = State.IDLE;
                }
                break;
        }

        slides.update();

        TelemetryUtil.packet.put("Deposit.state", this.state);
        TelemetryUtil.packet.put("Deposit.targetY", this.targetY);
        TelemetryUtil.packet.put("Deposit: Arm inPosition", arm.inPosition());
        TelemetryUtil.packet.put("Deposit: Slides inPosition", slides.inPosition(0.5));
        TelemetryUtil.packet.put("Deposit: Claw inPosition", arm.clawFinished());
    }

    public void moveToWithRad(double armTargetRad, double targetY){
        arm.setArmRotation(armTargetRad, 1.0);
        slides.setTargetLength(targetY);
    }

    public void moveToStart(){
        arm.setArmRotation(0.0, 1.0);
        arm.setClawRotation(0.0, 1.0);
        arm.sampleOpen();
        slides.setTargetLength(0.0);
    }

    private double targetY = speciHY;
    public void setDepositHeight(double targetY){
        this.targetY = Utils.minMaxClip(targetY, 0.0, 34.0);
    }

    public void setAuto(boolean auto){
        this.auto = auto;
    }

    public void setDepositHeightLowSample(){
        targetY = sampleLY;
        high = false;
    }

    public void setDepositHeightHighSample(){
        targetY = sampleHY;
        high = true;
    }

    public void setDepositLowSpeci(){
        targetY = speciLSY;
        high = false;
    }

    public void setDepositHighSpeci(){
        targetY = speciHY;
        high = true;
    }

//    public void setDepositLowSpeciEnd(){
//        targetY = speciLEY;
//        high = false;
//    }
//
//    public void setDepositHighSpeciEnd(){
//        targetY = speciHEY;
//        high = true;
//    }

    public double getDepositHeight(){
        return targetY;
    }

    public void prepareTransfer() {
        Log.i("FSM", this.state + ", prepareTransfer()");
        state = State.TRANSFER_PREPARE;
    }

    public void startTransfer() {
        Log.i("FSM", this.state + ", startTransfer()");
        if (state == State.TRANSFER_WAIT)
            state = State.TRANSFER_GRAB;
    }

    public boolean isSampleReady() {
        return state == State.HOLD;
    }

    public void startSampleDeposit() {
        Log.i("FSM", this.state + ", startSampleDeposit()");
        if (state == State.HOLD)
            state = State.SAMPLE_RAISE;
    }

    public void finishSampleDeposit() {
        Log.i("FSM", this.state + ", finishSampleDeposit()");
        if (state == State.SAMPLE_WAIT) {
            if (high) {
                setDepositHeightHighSample();
            } else {
                setDepositHeightLowSample();
            }
            state = State.SAMPLE_DEPOSIT;
        }
    }

    public boolean isSampleDepositDone() {
        return state == State.IDLE;
    }

    public void startOuttake() {
        Log.i("FSM", this.state + ", startOuttake()");
        if (state == State.HOLD || state == State.GRAB_HOLD)
            state = State.OUTTAKE_MOVE;
    }

    public boolean isOuttakeDone() {
        return state == State.IDLE;
    }

    public void startSpecimenGrab() {
        Log.i("FSM", this.state + ", startSpecimenGrab()");
        if (state == State.IDLE)
            state = State.GRAB_MOVE;
    }

    public void finishSpecimenGrab() {
        Log.i("FSM", this.state + ", finishSpecimenGrab()");
        if (state == State.GRAB_WAIT)
            state = State.GRAB;
    }

    public boolean isSpecimenReady() {
        return state == State.GRAB_HOLD;
    }

    public void startSpecimenDeposit() {
        Log.i("FSM", this.state + ", startSpecimenDeposit()");
        if(state == State.GRAB_HOLD)
            state = State.SPECI_RAISE;
    }

    public void finishSpecimenDeposit() {
        Log.i("FSM", this.state + ", finishSpecimenDeposit()");
//        if (state == State.SPECI_WAIT) {
//            if (high) {
//                setDepositHighSpeciEnd();
//            } else {
//                setDepositLowSpeciEnd();
//            }
//            state = State.SPECI_DEPOSIT;
//        }
        if(state == State.SPECI_RAISE)
            state = State.RELEASE;
    }

    public boolean isSpecimenDepositDone() {
        return state == State.IDLE;
    }

    public void retract() {
        Log.i("FSM", this.state + ", retract()");
        state = State.RETRACT;
    }

    public boolean isRetractDone(){
        return state == State.IDLE;
    }
}
