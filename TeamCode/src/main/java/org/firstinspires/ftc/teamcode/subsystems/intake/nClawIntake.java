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

    private double intakeSetTargetPos = 0;

    // turretBufferAng -> angle that allows for any rotation to occur with the turret still inside the robot. use in any retract/extend states

    public static double intakeTransferRot = 0, intakeGrabRot = 0.0;
    public static double turretBufferAng = 1.2506, turretRetractedAng = 2.7, turretSearchAng = Math.PI, turretTransferAng = 0.133, turretGrabAng = -Math.toRadians(15);
    public static double turretPreRot = 2.7415, turretSearchRot = 0, turretTransferRot = 1.857, turretGrabRot = 0.0;
    public static double turretPastSidePlatesRot = 1;
    public static double bufferMinExtension = 7;

    private boolean grab = false;
    private boolean sampleStatus = false;
    private boolean useCamera = true;
    private boolean finishTransferRequest = false;
    private Pose2d target;

    public enum State {
        START_EXTEND,
        MID_EXTEND,
        FULL_EXTEND,
        SEARCH,
        LOWER,
        GRAB_CLOSE,
        START_RETRACT,
        RETRACT,
        READY,
        TRANSFER_WAIT,
        TRANSFER_END,
        TEST
    }

    public State state = State.READY;

    public nClawIntake(Robot robot) {
        this.robot = robot;

        endAffector = new EndAffector(this.robot);

        intakeLight = robot.hardwareMap.get(DigitalChannel.class, "intakeLight");
        intakeLight.setMode(DigitalChannel.Mode.OUTPUT);
        intakeLight.setState(false);

        if (Globals.RUNMODE != RunMode.TELEOP) {
            resetExtendoEncoders();
        }

        intakeSetTargetPos = 15;

        target = new Pose2d(0, 0, 0);
    }

    //general update for entire class
    public void update() {
        endAffector.update();

        switch (state) {
            case START_EXTEND:
                // Pre-rotate the turret + claw servos
                endAffector.setIntakeExtension(bufferMinExtension);
                endAffector.setTurretAngle(turretBufferAng);
                endAffector.setTurretRot(turretPreRot);
                endAffector.setIntakeRotation(target.heading);

                endAffector.setClawState(grab);

                // Wait for extension past certain length or for the buffer ang to be reached, meaning we can full send rotation
                if (endAffector.extendoInPosition() || endAffector.turretAngInPosition()) {
                    if (intakeSetTargetPos > bufferMinExtension)
                        state = State.FULL_EXTEND;
                    else
                        state = State.MID_EXTEND;
                }
                break;
            case MID_EXTEND:
                // Get past the side plates so if our target position is less than min extension we won't clip
                endAffector.setIntakeExtension(bufferMinExtension);
                endAffector.setTurretAngle(turretBufferAng);
                endAffector.setTurretRot(turretPastSidePlatesRot);
                endAffector.setIntakeRotation(target.heading);

                if (endAffector.turretAngInPosition())
                    state = State.FULL_EXTEND;
                break;
            case FULL_EXTEND:
                // Fully extend + rotate to search positions
                endAffector.setIntakeExtension(intakeSetTargetPos);
                endAffector.setTurretAngle(turretSearchAng);
                endAffector.setTurretRot(turretSearchRot);
                endAffector.setIntakeRotation(target.heading);

                endAffector.setClawState(grab);

                // Wait for full extension and turret in position before starting search
                if (endAffector.extendoInPosition() && endAffector.turretRotInPosition()) {
                    state = useCamera ? State.SEARCH : State.LOWER;
                    intakeLight.setState(true);
                }
                break;
            case SEARCH:
                // Begin Serach, just hold positions
                endAffector.setIntakeExtension(intakeSetTargetPos);
                endAffector.setIntakeRotation(target.heading);
                endAffector.setTurretAngle(turretSearchAng);
                endAffector.setTurretRot(turretSearchRot);

                endAffector.setClawState(false);

                // Use grab as a flag for whether or not a block has been found
                if (grab && endAffector.rotInPosition()) {
                    state = State.LOWER;
                }
                break;
            case LOWER:
                // intakeAt method should hopefully calculate new extension, new turretAngle + Rotation, and move in to grab
                if (useCamera) {
                    endAffector.intakeAt(target);
                } else {
                    endAffector.setIntakeExtension(intakeSetTargetPos);
                    endAffector.setIntakeRotation(intakeGrabRot);
                    endAffector.setTurretAngle(turretGrabAng);
                    endAffector.setTurretRot(turretGrabRot);
                }

                endAffector.setClawState(false);

                // everything in position before grabbing
                if (endAffector.inPosition()) {
                    state = State.GRAB_CLOSE;
                }
                break;
            case GRAB_CLOSE:
                // Grab the block and wait for a confirmtion that we have the block
                if (useCamera) {
                    endAffector.intakeAt(target);
                } else {
                    endAffector.setIntakeExtension(intakeSetTargetPos);
                    endAffector.setIntakeRotation(intakeGrabRot);
                    endAffector.setTurretAngle(turretGrabAng);
                    endAffector.setTurretRot(turretGrabRot);
                }

                // grab
                endAffector.setClawState(true);

                // begin retract once grab finished
                if (sampleStatus)
                    state = State.START_RETRACT;
                break;
            case START_RETRACT:
                // Get the arm in a proper angle for a full retract
                endAffector.setIntakeExtension(bufferMinExtension);
                endAffector.setIntakeRotation(intakeTransferRot);
                endAffector.setTurretAngle(turretBufferAng);
                endAffector.setTurretRot(turretPastSidePlatesRot);

                endAffector.setClawState(true);

                // if grab failed go back to search
                if (endAffector.turretAngInPosition()) {
                    // If we have a sample, transfer otherwise just retract into it
                    if (sampleStatus)
                        state = State.TRANSFER_WAIT;
                    else
                        state = State.RETRACT;
                }
                break;
            case RETRACT:
                // full retract into transfer
                endAffector.setIntakeExtension(0.0);
                endAffector.setIntakeRotation(intakeTransferRot);
                endAffector.setTurretAngle(turretRetractedAng);
                endAffector.setTurretRot(turretTransferRot);

                endAffector.setClawState(false);

                // true grab -> holding a sample
                if (endAffector.extendoInPosition()) {
                    state = State.READY;
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
                if (endAffector.inPosition() && finishTransferRequest) {
                    state = State.TRANSFER_END;
                    finishTransferRequest = false;
                    sampleStatus = false;
                }
                break;
            case TRANSFER_END:
                endAffector.setIntakeExtension(0.0);
                endAffector.setIntakeRotation(intakeTransferRot);
                endAffector.setTurretAngle(turretTransferAng);
                endAffector.setTurretRot(turretTransferRot);

                endAffector.setClawState(false);

                // once the grab is finished, send back to RETRACT. false grab changes from HOLD to READY
                // no need to worry about whacking stuff b/c both states require rotation to be in the turretTransferRot value
                if (endAffector.grabInPosition()) {
                    state = State.RETRACT;
                    grab = false;
                }
                break;

            case TEST:
                endAffector.setIntakeExtension(intakeSetTargetPos);
                break;
        }

        updateTelemetry();
    }

    public void useCamera(boolean val) {
        useCamera = val;
    }

    // TODO: Determine minMaxClips for setters
    public void setTurretGrabRot(double t) {
        turretGrabRot = t;
    }

    public void finishTransfer() {
        finishTransferRequest = state == State.TRANSFER_WAIT;
    }


    public void setClawRotation(double angle) {
        if (angle > 1.7) {
            angle = -1.7;
        } else if (angle < -1.7) {
            angle = 1.7;
        }
        intakeGrabRot = angle;
    }

    public double getClawRotAngle() {
        return intakeGrabRot;
    }

    public void setTargetPose(Pose2d t) {
        target = t.clone();
    }

    public void extend() {
        if (this.state == State.READY)
            this.state = State.START_EXTEND;
    }

    public boolean isExtended() {
        return state == State.SEARCH || state == State.LOWER || state == State.GRAB_CLOSE;
    }

    public void retract() {
        this.state = State.START_RETRACT;
    }

    public boolean isRetracted() {
        return state == State.READY;
    }

    public void grab(boolean closed) {
        grab = closed;
        if (!grab)
            state = State.SEARCH;
    }

    // Confirm pickup
    public void confirmGrab() {
        sampleStatus = true;
    }

    public boolean hasSample() {
        return sampleStatus;
    }

    public boolean isTransferReady() {
        return endAffector.inPosition() && state == State.TRANSFER_WAIT;
    }

    /*public void release() {
        if (this.state == State.HOLD) {
            this.state = State.READY;
            grab = false;
        }
    }*/

    //public boolean dropFinished() {
    //    return state == State.SEARCH;
    //}

    public void setIntakeLength(double targetPos) {
        this.intakeSetTargetPos = Utils.minMaxClip(targetPos, 1, 19);
    }

    public double getIntakeTargetPos() {
        return this.intakeSetTargetPos;
    }

//    public void forcePullIn() { forcePull = true; }

    public void updateTelemetry() {
        TelemetryUtil.packet.put("ClawIntake.clawRotationAlignAngle", intakeGrabRot);
        LogUtil.intakeClawRotationAngle.set(intakeGrabRot);
        TelemetryUtil.packet.put("ClawIntake.grab", grab);
        LogUtil.intakeClawGrab.set(grab);
        TelemetryUtil.packet.put("ClawIntake clawRotation angle", endAffector.getIntakeRotation());
        TelemetryUtil.packet.put("ClawIntake State", this.state);
        LogUtil.intakeState.set(this.state.toString());
    }

    public void resetExtendoEncoders() {
        Log.e("RESETTTING", "RESTETING EXTENDO *************");
        endAffector.intakeExtension.resetExtendoEncoders();
    }
}
