package org.firstinspires.ftc.teamcode.subsystems.deposit;

import android.util.Log;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.utils.Globals;
import org.firstinspires.ftc.teamcode.utils.LogUtil;
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
        SAMPLE_RELEASE,
        SAMPLE_FINISH,
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
        RETRACT,
        TEST
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
    public static double intakeWaitRad = 0.4, intakeWaitY = 0.0, intakeWaitClawRad = 1.235;
    // transfer positions, move in to grab
    public static double intakeRad = 0.0266, intakeY = 0.0, intakeClawRad = 1.4;
    // moving positions with a sample
    public static double sampleHoldRad = 0.0, holdY = 0.0, sampleHoldClawRad = -Math.PI / 2;
    public static double specimenGrabRad = 0.0, specimenGrabClawRad = 0.0, specimenConfirmRad = Math.toRadians(40), specimenConfirmClawRad = Math.toRadians(40);
    // sample basket positions
    public static double sampleLY = 16.75, sampleHY = 31, sampleRaiseRad = Math.toRadians(90), sampleDepositRad = 2.2, sampleDepositClawRad = -0.2;
    // outtake positions, drop behind robot
    public static double outtakeRad = Math.toRadians(180), outtakeY = 0.0, outtakeClawRad = 0.0;
    // grabbing positions, holdGrab -> off the wall, grabRetract --> moving with a specimen
    // specimen chamber positions
    public static double speciLSY = 19.4;
    public static double  speciHRad = 2.5, speciHClawRad = -1.5, speciHY = 20 ;
    // TODO: ^ These values look about fine tbh, just had to reverse the sign of the claw. Need to test

    private long currentTime = -1;
    private long sampleReleaseTime = -1;
    private long specimenReleaseTime = -1;
    public static int sampleReleaseDuration = 300;
    public static int specimenReleaseDuration = 500;
    private long grabStartTime = -1;
    public static int transferBufferDuration = 200;

    private boolean high = true;

    public enum HangMode {
        OUT,
        PULL,
        OFF
    }
    public HangMode hangMode = HangMode.OFF;
    public boolean holdSlides = false;

    public Deposit(Robot robot){
        this.robot = robot;
        this.slides = new Slides(this.robot);

        arm = new Arm(this.robot);

        state = Globals.hasSpecimenPreload ? State.GRAB_HOLD : Globals.hasSamplePreload ? State.HOLD : State.RETRACT;
    }

    public void update(){
        this.currentTime = System.nanoTime();

        switch (state) {
            case IDLE:
                moveToStart();
                break;
            case TRANSFER_PREPARE:
                moveToWithRad(intakeWaitRad, intakeWaitY);
                arm.setClawRotation(intakeWaitClawRad, 1.0);
                arm.sampleOpen();

                if(arm.inPosition() && slides.inPosition(1) && arm.clawFinished()){
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
                arm.speciOpen();

                if(arm.inPosition() && slides.inPosition(1)){ // Need to figure out this threshold better
                    state = State.TRANSFER_CLOSE;
                    arm.clawClose();
                    this.grabStartTime = this.currentTime;
                }
                break;
            case TRANSFER_CLOSE:
                moveToWithRad(intakeRad, intakeY);
                arm.setClawRotation(intakeClawRad, 1.0);
                arm.clawClose();

                if (arm.clawFinished() && this.currentTime - this.grabStartTime >= transferBufferDuration * 1e6) {
                    state = State.TRANSFER_FINISH;
                    robot.clawIntake.release();
                }
                break;
            case TRANSFER_FINISH:
                moveToWithRad(sampleHoldRad, holdY);
                arm.setClawRotation(sampleHoldClawRad, 1.0);
                arm.clawClose();

                if(arm.inPosition()){
                    state = State.HOLD;
                }
                break;
            case HOLD:
                moveToWithRad(sampleHoldRad, holdY);
                arm.setClawRotation(sampleHoldClawRad, 1.0);
                arm.clawClose();
                break;
            case SAMPLE_RAISE:
                moveToWithRad(sampleRaiseRad, targetY);
                arm.setClawRotation(sampleDepositClawRad, 1.0);

                if(arm.inPosition() && slides.inPosition(2)){
                    moveToWithRad(sampleDepositRad, targetY);
                    state = State.SAMPLE_WAIT;
                }
                break;
            case SAMPLE_WAIT:
                moveToWithRad(sampleDepositRad, targetY);
                arm.setClawRotation(sampleDepositClawRad, 1.0);
                this.sampleReleaseTime = this.currentTime;

                break;
            case SAMPLE_RELEASE:
                moveToWithRad(sampleDepositRad, targetY);
                arm.setClawRotation(sampleDepositClawRad, 1.0);
                arm.sampleOpen();

                if (arm.clawFinished() && this.currentTime >= this.sampleReleaseTime + sampleReleaseDuration * 1e6) {
                    state = State.SAMPLE_FINISH;
                }
                break;
            case SAMPLE_FINISH:
                moveToWithRad(0, targetY);
                arm.setClawRotation(0, 1.0);

                if (arm.armRotation.getCurrentAngle() <= Math.toRadians(90)) {
                    state = State.RETRACT;
                }
                break;
            case OUTTAKE_MOVE:
                moveToWithRad(outtakeRad, outtakeY);
                arm.setClawRotation(outtakeClawRad, 1.0);

                if(arm.inPosition() && slides.inPosition(1)){
                    state = State.OUTTAKE_RELEASE;
                }
                break;
            case OUTTAKE_RELEASE:
                moveToWithRad(outtakeRad, outtakeY);
                arm.setClawRotation(outtakeClawRad, 1.0);
                arm.sampleOpen();

                if (arm.clawFinished()) {
                    state = State.RETRACT;
                }
                break;
            case GRAB_MOVE:
                moveToWithRad(specimenGrabRad, holdY);
                arm.setClawRotation(specimenGrabClawRad, 1.0);

                if (arm.inPosition() && slides.inPosition(1)) {
                    state = State.GRAB_WAIT;
                    arm.speciOpen();
                }
                break;
            case GRAB_WAIT:
                moveToWithRad(specimenGrabRad, holdY);
                arm.setClawRotation(specimenGrabClawRad, 1.0);
                break;
            case GRAB:
                moveToWithRad(specimenGrabRad, holdY);
                arm.setClawRotation(specimenGrabClawRad, 1.0);
                arm.clawClose();

                if (arm.clawFinished()) {
                    state = State.GRAB_RETRACT;
                }
                break;
            case GRAB_RETRACT:
                moveToWithRad(specimenConfirmRad, holdY);
                arm.setClawRotation(specimenConfirmClawRad, 1.0);

                if (arm.inPosition()) {
                    state = State.GRAB_HOLD;
                }
                break;
            case GRAB_HOLD:
                moveToWithRad(specimenConfirmRad, holdY);
                arm.setClawRotation(specimenConfirmClawRad, 1.0);
                targetY = speciHY;

                break;
            case SPECI_RAISE:
                moveToWithRad(speciHRad, targetY);
                arm.setClawRotation(speciHClawRad, 1.0);

                if (arm.inPosition() && arm.clawFinished()) {
                    state = State.SPECI_DEPOSIT;
                }
                break;
            case SPECI_DEPOSIT:
                moveToWithRad(speciHRad, targetY);
                arm.setClawRotation(speciHClawRad, 1.0);
                this.specimenReleaseTime = this.currentTime;

                break;
            case RELEASE:
                arm.speciOpen();

                if (arm.clawFinished() && this.currentTime >= this.specimenReleaseTime + specimenReleaseDuration * 1e6) {
                    state = State.RETRACT;
                }
                break;
            case RETRACT:
                moveToStart();

                if(arm.inPosition() && slides.inPosition(1)){
                    state = State.IDLE;
                }
                break;
            case TEST:
                break;
        }

        if (holdSlides) {
            slides.setTargetLength(targetY);
        }

        slides.update();

        if (hangMode == HangMode.PULL) {
            slides.setTargetPowerFORCED(-1.0);
            targetY = slides.getLength() - 0.5;
        } else if (hangMode == HangMode.OUT) {
            slides.setTargetPowerFORCED(0.7);
            targetY = slides.getLength() + 0.5;
        }
        if (hangMode != HangMode.OFF) holdSlides = true;

        TelemetryUtil.packet.put("Deposit.state", this.state);
        LogUtil.depositState.set(this.state.toString());
        TelemetryUtil.packet.put("Deposit.targetY", this.targetY);
        TelemetryUtil.packet.put("Deposit: Arm inPosition", arm.inPosition());
        TelemetryUtil.packet.put("Deposit: Slides inPosition", slides.inPosition(0.5));
        TelemetryUtil.packet.put("Deposit: Claw inPosition", arm.clawFinished());
        TelemetryUtil.packet.put("Deposit: Hanging", hangMode);

        hangMode = HangMode.OFF;
    }

    public void moveToWithRad(double armTargetRad, double targetY){
        arm.setArmRotation(armTargetRad, 1.0);
        slides.setTargetLength(targetY);
    }

    public void moveToStart(){
        arm.setArmRotation(0.001, 1.0);
        arm.setClawRotation(0.001, 1.0);
        arm.sampleOpen();
        slides.setTargetLength(0.0);
    }

    private double targetY = speciHY;
    public void setDepositHeight(double target){
        this.targetY = Utils.minMaxClip(target, 0.0, Slides.maxSlidesHeight);
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

    public double getDepositHeight(){
        return targetY;
    }

    public void prepareTransfer() {
        Log.i("FSM", this.state + ", prepareTransfer()");
        state = State.TRANSFER_PREPARE;
    }

    public void startTransfer() {
        Log.i("FSM", this.state + ", startTransfer()");
        if (state == State.TRANSFER_PREPARE || state == State.TRANSFER_WAIT) state = State.TRANSFER_GRAB;
    }

    public boolean isSampleReady() {
        return state == State.HOLD;
    }

    public void startSampleDeposit() {
        Log.i("FSM", this.state + ", startSampleDeposit()");
        if (state == State.HOLD) state = State.SAMPLE_RAISE;
    }

    public boolean isSampleUp() { return state == State.SAMPLE_WAIT && arm.inPosition(); }

    public void finishSampleDeposit() {
        Log.i("FSM", this.state + ", finishSampleDeposit()");
        if (state == State.SAMPLE_WAIT || state == State.SAMPLE_RAISE) {
/*
            if (high) {
                setDepositHeightHighSample();
            } else {
                setDepositHeightLowSample();
            }
 */
            state = State.SAMPLE_RELEASE;
        }
    }

    public static double heightThresh = 12.0;
    public boolean safeToMove(){
        return slides.getLength() <= heightThresh;
    }

    public boolean isSampleDepositDone() {
        return state == State.IDLE || state == State.RETRACT;
    }

    public void startOuttake() {
        Log.i("FSM", this.state + ", startOuttake()");
        if (state == State.HOLD || state == State.GRAB_HOLD) state = State.OUTTAKE_MOVE;
    }

    public boolean isOuttakeDone() {
        return state == State.IDLE;
    }

    public void startSpecimenGrab() {
        Log.i("FSM", this.state + ", startSpecimenGrab()");
        if (state == State.IDLE || state == State.GRAB_HOLD) state = State.GRAB_MOVE;
    }

    public void finishSpecimenGrab() {
        Log.i("FSM", this.state + ", finishSpecimenGrab()");
        if (state == State.GRAB_WAIT) state = State.GRAB;
    }

    public boolean isSpecimenReady() {
        return state == State.GRAB_HOLD;
    }

    public void startSpecimenDeposit() {
        Log.i("FSM", this.state + ", startSpecimenDeposit()");
        if (state == State.GRAB_HOLD) state = State.SPECI_RAISE;
    }

    public boolean readyToRam(){
        return state == State.SPECI_DEPOSIT;
    }

    public void finishSpecimenDeposit() {
        Log.i("FSM", this.state + ", finishSpecimenDeposit()");
        if (state == State.SPECI_RAISE || state == State.SPECI_DEPOSIT) state = State.RELEASE;
    }

    public boolean isSpecimenDepositDone() {
        return state == State.IDLE;
    }

    public void retract() {
        Log.i("FSM", this.state + ", retract()");
        state = State.RETRACT;
    }

    public boolean isRetractDone(){
        //Log.i("funny stuffs", "Current: " + arm.armRotation.getCurrentAngle());
        //Log.i("funny stuffs", "Target: " + arm.armRotation.getTargetAngle());
        return state == State.IDLE;
    }
}
