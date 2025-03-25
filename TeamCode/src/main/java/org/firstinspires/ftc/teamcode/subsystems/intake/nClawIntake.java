package org.firstinspires.ftc.teamcode.subsystems.intake;

import android.util.Log;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DigitalChannel;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.utils.Globals;
import org.firstinspires.ftc.teamcode.utils.LogUtil;
import org.firstinspires.ftc.teamcode.utils.Pose2d;
import org.firstinspires.ftc.teamcode.utils.RunMode;
import org.firstinspires.ftc.teamcode.utils.TelemetryUtil;
import org.firstinspires.ftc.teamcode.utils.Utils;

@Config
public class nClawIntake {
    private final Robot robot;

    private final EndAffector endAffector;

    private final DigitalChannel intakeLight;

    private double intakeSetTargetPos;

    // turretBufferAng -> angle that allows for any rotation to occur with the turret still inside the robot. use in any retract/extend states

    public static double intakeTransferRot = 0.0, intakeGrabRot = 0.0;
    public static double turretBufferAng = Math.toRadians(75), turretRetractedAng = Math.toRadians(125), turretSearchAng = Math.toRadians(45), turretTransferAng = Math.toRadians(60), turretGrabAng = -Math.toRadians(15);
    public static double turretPreRot = Math.toRadians(15), turretSearchRot = Math.toRadians(180), turretTransferRot = 0.0, turretGrabRot = 0.0;

    private boolean grab = false;
    private boolean useCamera = true;
    private boolean prepareTransfer = false, completeTransfer = false;

    public enum nClawIntakeState {
        START_EXTEND,
        FULL_EXTEND,
        SEARCH,
        LOWER,
        GRAB_CLOSE,
        CONFIRM,
        RETRACT_BUFFER,
        RETRACT,
        HOLD,
        READY,
        TRANSFER_WAIT,
        TRANSFER_END,
        TEST
    }

    public nClawIntakeState clawIntakeState = nClawIntakeState.READY;

    public nClawIntake(Robot robot) {
        this.robot = robot;

        endAffector = new EndAffector(this.robot);

        intakeLight = robot.hardwareMap.get(DigitalChannel.class, "intakeLight");
        intakeLight.setMode(DigitalChannel.Mode.OUTPUT);
        intakeLight.setState(false);

        if (Globals.RUNMODE != RunMode.TELEOP) {
            resetExtendoEncoders();
        }

        this.intakeSetTargetPos = 15;
    }

    //general update for entire class
    public void update() {
        endAffector.update();

        switch (clawIntakeState) {
            case START_EXTEND:
                // Pre-rotate the turret + claw servos
                intakeSetTargetPos = 2.0;
                endAffector.setIntakeExtension(intakeSetTargetPos);
                endAffector.setTurretAngle(turretBufferAng);
                endAffector.setTurretRot(turretPreRot);
                endAffector.setIntakeRotation(intakeTransferRot);

                endAffector.setClawState(grab);

                // Wait for extension past certain length or for the buffer ang to be reached, meaning we can full send rotation
                if (endAffector.extendoInPosition() || endAffector.turretAngInPosition()) {
                    clawIntakeState = nClawIntakeState.FULL_EXTEND;
                }
                break;
            case FULL_EXTEND:
                // Fully extend + rotate to search positions
                endAffector.setIntakeExtension(intakeSetTargetPos);
                endAffector.setTurretAngle(turretSearchAng);
                endAffector.setTurretRot(turretSearchRot);
                endAffector.setIntakeRotation(intakeTransferRot);

                endAffector.setClawState(grab);

                // Wait for full extension and turret in position before starting search
                if (endAffector.extendoInPosition() && endAffector.turretRotInPosition()) {
                    clawIntakeState = useCamera ? nClawIntakeState.SEARCH : nClawIntakeState.LOWER;
                    intakeLight.setState(true);
                }
                break;
            case SEARCH:
                // Begin Serach, just hold positions
                endAffector.setIntakeExtension(intakeSetTargetPos);
                endAffector.setIntakeRotation(intakeTransferRot);
                endAffector.setTurretAngle(turretSearchAng);
                endAffector.setTurretRot(turretSearchRot);

                endAffector.setClawState(false);

                // Use grab as a flag for whether or not a block has been found
                if (this.grab && endAffector.rotInPosition()) {
                    clawIntakeState = nClawIntakeState.LOWER;
                }
                break;
            case LOWER:
                // intakeAt method should hopefully calculate new extension, new turretAngle + Rotation, and move in to grab
                if(useCamera){
                    intakeAt();
                }else{
                    endAffector.setIntakeExtension(intakeSetTargetPos);
                    endAffector.setIntakeRotation(intakeGrabRot);
                    endAffector.setTurretAngle(turretGrabAng);
                    endAffector.setTurretRot(turretGrabRot);
                }

                endAffector.setClawState(false);

                // everything in position before grabbing
                if (endAffector.inPosition()){
                    clawIntakeState = nClawIntakeState.GRAB_CLOSE;
                }
                break;
            case GRAB_CLOSE:
                // hold prev position
                if(useCamera){
                    intakeAt();
                }else{
                    endAffector.setIntakeExtension(intakeSetTargetPos);
                    endAffector.setIntakeRotation(intakeGrabRot);
                    endAffector.setTurretAngle(turretGrabAng);
                    endAffector.setTurretRot(turretGrabRot);
                }

                // grab
                endAffector.setClawState(true);

                // begin retract once grab finished
                if (endAffector.grabInPosition()) {
                    clawIntakeState = nClawIntakeState.CONFIRM;
                }
                break;
            case CONFIRM:
                // begin moving into transfer position while holding same extendo
                // keeping this maybe so in teleop drivers can try again if grab missed
                // or somehow got wrong color
                endAffector.setIntakeExtension(intakeSetTargetPos);
                endAffector.setIntakeRotation(intakeTransferRot);
                endAffector.setTurretAngle(turretBufferAng);
                endAffector.setTurretRot(turretTransferRot);

                endAffector.setClawState(true);

                // if grab failed go back to search
                if (!this.grab) {
                    clawIntakeState = nClawIntakeState.SEARCH;
                }
                break;
            case RETRACT_BUFFER:
                // begin retraction upon being prompted, have a 6in buffer should be enough for swinging stuff around
                endAffector.setIntakeExtension(2.0);
                endAffector.setIntakeRotation(intakeTransferRot);
                endAffector.setTurretAngle(turretBufferAng);
                endAffector.setTurretRot(turretTransferRot);

                endAffector.setClawState(grab);

                // once turret is in place we can safe retract
                if(endAffector.turretAngInPosition()){
                    clawIntakeState = nClawIntakeState.RETRACT;
                }
                break;
            case RETRACT:
                // full retract into transfer
                endAffector.setIntakeExtension(0.0);
                endAffector.setIntakeRotation(intakeTransferRot);
                endAffector.setTurretAngle(turretRetractedAng);
                endAffector.setTurretRot(turretTransferRot);

                endAffector.setClawState(grab);

                // true grab -> holding a sample
                if (endAffector.extendoInPosition()) {
                    clawIntakeState = this.grab ? nClawIntakeState.HOLD : nClawIntakeState.READY;
                    this.intakeLight.setState(false);
                }
                break;
            case READY:
                // hold in start position, everything tucked in while moving so defense can be played. no sample ver
                endAffector.setIntakeExtension(0.0);
                endAffector.setIntakeRotation(intakeTransferRot);
                endAffector.setTurretAngle(turretRetractedAng);
                endAffector.setTurretRot(turretTransferRot);

                endAffector.setClawState(false);
                break;
            case HOLD:
                // tucked in with sample
                endAffector.setIntakeExtension(0.0);
                endAffector.setIntakeRotation(intakeTransferRot);
                endAffector.setTurretAngle(turretRetractedAng);
                endAffector.setTurretRot(turretTransferRot);

                endAffector.setClawState(true);

                // Prepare transfer can only be set true in HOLD, used to send intake turret into transfer position
                if(prepareTransfer){
                    clawIntakeState = nClawIntakeState.TRANSFER_WAIT;
                    prepareTransfer = false;
                }
                break;
            case TRANSFER_WAIT:
                // hold in transfer position
                endAffector.setIntakeExtension(0.0);
                endAffector.setIntakeRotation(intakeTransferRot);
                endAffector.setTurretAngle(turretTransferAng);
                endAffector.setTurretRot(turretTransferRot);

                endAffector.setClawState(true);

                // Complete transfer can only be called in TRANSFER_WAIT, must have everything correct
                // used to release intake grip on sample, should be called in deposit after the deposit has a firm grip
                // TODO: check endAffector.inPosition()
                if(endAffector.inPosition() && completeTransfer){
                    clawIntakeState = nClawIntakeState.TRANSFER_END;
                    completeTransfer = false;
                }
                break;
            case TRANSFER_END:
                endAffector.setIntakeExtension(0.0);
                endAffector.setIntakeRotation(intakeTransferRot);
                endAffector.setTurretAngle(turretTransferAng);
                endAffector.setTurretAngle(turretTransferRot);

                endAffector.setClawState(false);

                // once the grab is finished, send back to RETRACT. false grab changes from HOLD to READY
                // no need to worry about whacking stuff b/c both states require rotation to be in the turretTransferRot value
                if(endAffector.grabInPosition()){
                    clawIntakeState = nClawIntakeState.RETRACT;
                    grab = false;
                }
                break;

            case TEST:
                endAffector.setIntakeExtension(intakeSetTargetPos);
                break;
        }

        updateTelemetry();
    }

    // TODO: Determine minMaxClips for setters
    public void setTurretGrabRot(double t){
        turretGrabRot = t;
    }

    public void setCamera(boolean use){
        useCamera = use;
    }

    public void prepareIntakeTransfer(){
        if(clawIntakeState == nClawIntakeState.HOLD){
            prepareTransfer = true;
        }
    }

    public void completeIntakeTransfer(){
        if(clawIntakeState == nClawIntakeState.TRANSFER_WAIT){
            completeTransfer = true;
        }
    }

    public void setClawRotation(double angle) {
        if (angle > 1.7) {
            angle = -1.7;
        } else if (angle < -1.7){
            angle = 1.7;
        }
        intakeGrabRot = angle;
    }

    public double getClawRotAngle() {
        return intakeGrabRot;
    }

    private Pose2d target;
    public void setTargetPose(Pose2d t){
        target = t.clone();
    }

    private void intakeAt(){
        endAffector.intakeAt(target);
    }

    public void extend() {
        if (this.clawIntakeState == nClawIntakeState.READY || this.clawIntakeState == nClawIntakeState.HOLD)
            this.clawIntakeState = nClawIntakeState.START_EXTEND;
    }

    public boolean isExtended() {
        return this.clawIntakeState == nClawIntakeState.SEARCH || this.clawIntakeState == nClawIntakeState.LOWER
                || this.clawIntakeState == nClawIntakeState.GRAB_CLOSE || this.clawIntakeState == nClawIntakeState.CONFIRM;
    }

    public void retract() {
        this.clawIntakeState = nClawIntakeState.RETRACT;
    }

    public boolean isRetracted() {
        return (this.clawIntakeState == nClawIntakeState.HOLD || this.clawIntakeState == nClawIntakeState.READY);
    }

    public void grab(boolean closed) {
        grab = closed;
    }

    public boolean hasSample() { return this.grab; }

    public boolean grabFinished() {
        return clawIntakeState == nClawIntakeState.CONFIRM;
    }

    public void release() {
        if (this.clawIntakeState == nClawIntakeState.HOLD) {
            this.clawIntakeState = nClawIntakeState.READY;
            grab = false;
        }
    }

    public boolean dropFinished() {
        return clawIntakeState == nClawIntakeState.SEARCH;
    }

    public void setIntakeTargetPos(double targetPos) {
        this.intakeSetTargetPos = Utils.minMaxClip(targetPos, 1, 19);
    }

    public double getIntakeTargetPos() {
        return this.intakeSetTargetPos;
    }

//    public void forcePullIn() { forcePull = true; }

    public void updateTelemetry(){
        TelemetryUtil.packet.put("ClawIntake.clawRotationAlignAngle", intakeGrabRot);
        LogUtil.intakeClawRotationAngle.set(intakeGrabRot);
        TelemetryUtil.packet.put("ClawIntake.grab", grab);
        LogUtil.intakeClawGrab.set(grab);
        TelemetryUtil.packet.put("ClawIntake clawRotation angle", endAffector.getIntakeRotation());
        TelemetryUtil.packet.put("ClawIntake State", this.clawIntakeState);
        LogUtil.intakeState.set(this.clawIntakeState.toString());
    }

    public void resetExtendoEncoders() {
        Log.e("RESETTTING", "RESTETING EXTENDO *************");
        endAffector.intakeExtension.resetExtendoEncoders();
    }
}
