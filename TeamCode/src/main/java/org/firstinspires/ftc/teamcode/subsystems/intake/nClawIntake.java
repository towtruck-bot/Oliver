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

    private final IntakeTurret intakeTurret;

    private final DigitalChannel intakeLight;

    private double intakeSetTargetPos = 0;

    // turretBufferAng -> angle that allows for any rotation to occur with the turret still inside the robot. use in any retract/extend states

    public static double transferRotation = 0, grabRotation = 0.0;
    public static double turretBufferAngle = 1.2506, turretRetractedAngle = 2.7, turretSearchAngle = Math.PI, turretTransferAngle = 0.133, turretGrabAngle = -Math.toRadians(15);
    public static double turretPreRotation = 0.7, turretSearchRotation = 3.14, turretTransferRotation = 0, turretGrabRotation = 0.0;
    public static double turretPastSidePlatesRotation = 1.7;
    public static double minExtension = 7;
    public static double lowerDelay = 250;

    private boolean grab = false;
    private boolean sampleStatus = false;
    private boolean useCamera = true;
    private boolean finishTransferRequest = false;
    public Pose2d target;
    private long lowerStart = 0;

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

        intakeTurret = new IntakeTurret(this.robot);

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
        switch (state) {
            case START_EXTEND:
                // Pre-rotate the turret + claw servos
                intakeTurret.setIntakeExtension(minExtension);
                intakeTurret.setTurretArmTarget(turretBufferAngle);
                intakeTurret.setTurretRotation(turretPreRotation);
                intakeTurret.setClawRotation(target.heading);

                intakeTurret.setClawState(grab);

                // Wait for extension past certain length or for the buffer ang to be reached, meaning we can full send rotation
                if (intakeTurret.extendoInPosition() || intakeTurret.turretAngInPosition()) {
                    if (intakeSetTargetPos > minExtension)
                        state = State.FULL_EXTEND;
                    else
                        state = State.MID_EXTEND;
                }
                break;
            case MID_EXTEND:
                // Get past the side plates so if our target position is less than min extension we won't clip
                intakeTurret.setIntakeExtension(minExtension);
                intakeTurret.setTurretArmTarget(turretBufferAngle);
                intakeTurret.setTurretRotation(turretPastSidePlatesRotation);
                intakeTurret.setClawRotation(target.heading);

                if (intakeTurret.turretAngInPosition())
                    state = State.FULL_EXTEND;
                break;
            case FULL_EXTEND:
                // Fully extend + rotate to search positions
                intakeTurret.setIntakeExtension(intakeSetTargetPos);
                intakeTurret.setTurretArmTarget(turretSearchAngle);
                intakeTurret.setTurretRotation(turretSearchRotation);
                intakeTurret.setClawRotation(target.heading);

                intakeTurret.setClawState(grab);

                // Wait for full extension and turret in position before starting search
                if (intakeTurret.extendoInPosition() && intakeTurret.turretRotInPosition()) {
                    if (useCamera)
                        state = State.SEARCH;
                    else {
                        lowerStart = System.currentTimeMillis();
                        state = State.LOWER;
                    }
                    intakeLight.setState(true);
                }
                break;
            case SEARCH:
                // Begin Serach, just hold positions
                intakeTurret.setIntakeExtension(intakeSetTargetPos);
                intakeTurret.setClawRotation(target.heading);
                intakeTurret.setTurretArmTarget(turretSearchAngle);
                intakeTurret.setTurretRotation(turretSearchRotation);

                intakeTurret.setClawState(false);

                // Use grab as a flag for whether or not a block has been found
                if (grab && intakeTurret.rotInPosition()) {
                    lowerStart = System.currentTimeMillis();
                    state = State.LOWER;
                }
                break;
            case LOWER:
                // intakeAt method should hopefully calculate new extension, new turretAngle + Rotation, and move in to grab
                if (useCamera) {
                    intakeTurret.intakeAt(target);
                } else {
                    intakeTurret.setIntakeExtension(intakeSetTargetPos);
                    intakeTurret.setClawRotation(grabRotation);
                    intakeTurret.setTurretArmTarget(turretGrabAngle);
                    intakeTurret.setTurretRotation(turretGrabRotation);
                }

                intakeTurret.setClawState(false);

                // everything in position before grabbing
                if (intakeTurret.inPosition() && intakeTurret.extendoInPosition() && (System.currentTimeMillis() - lowerStart) > lowerDelay) {
                    state = State.GRAB_CLOSE;
                }
                break;
            case GRAB_CLOSE:
                // Grab the block and wait for a confirmtion that we have the block
                if (useCamera) {
                    intakeTurret.intakeAt(target);
                } else {
                    intakeTurret.setIntakeExtension(intakeSetTargetPos);
                    intakeTurret.setClawRotation(grabRotation);
                    intakeTurret.setTurretArmTarget(turretGrabAngle);
                    intakeTurret.setTurretRotation(turretGrabRotation);
                }

                // grab
                intakeTurret.setClawState(true);

                // begin retract once grab finished
                if (sampleStatus) {
                    state = State.START_RETRACT;
                }
                break;
            case START_RETRACT:
                // Get the arm in a proper angle for a full retract
                intakeTurret.setIntakeExtension(minExtension);
                intakeTurret.setClawRotation(transferRotation);
                intakeTurret.setTurretArmTarget(turretBufferAngle);
                intakeTurret.setTurretRotation(turretPastSidePlatesRotation);

                intakeTurret.setClawState(grab);

                // if grab failed go back to search
                if (intakeTurret.turretAngInPosition()) {
                    // If we have a sample, transfer otherwise just retract into it
                    if (sampleStatus)
                        state = State.TRANSFER_WAIT;
                    else {
                        state = State.RETRACT;
                        grab = false;
                    }
                }
                break;
            case RETRACT:
                // full retract into transfer
                intakeTurret.setIntakeExtension(0.0);
                intakeTurret.setClawRotation(transferRotation);
                intakeTurret.setTurretArmTarget(turretRetractedAngle);
                intakeTurret.setTurretRotation(turretTransferRotation);

                intakeTurret.setClawState(false);

                // true grab -> holding a sample
                if (intakeTurret.extendoInPosition()) {
                    state = State.READY;
                    this.intakeLight.setState(false);
                }
                break;
            case READY:
                // hold in start position, everything tucked in while moving so defense can be played. no sample ver
                intakeTurret.setIntakeExtension(0.0);
                intakeTurret.setClawRotation(transferRotation);
                intakeTurret.setTurretArmTarget(turretRetractedAngle);
                intakeTurret.setTurretRotation(turretTransferRotation);

                intakeTurret.setClawState(false);
                break;
            case TRANSFER_WAIT:
                // hold in transfer position
                intakeTurret.setIntakeExtension(0.0);
                intakeTurret.setClawRotation(transferRotation);
                intakeTurret.setTurretArmTarget(turretTransferAngle);
                intakeTurret.setTurretRotation(turretTransferRotation);

                intakeTurret.setClawState(true);

                // Complete transfer can only be called in TRANSFER_WAIT, must have everything correct
                // used to release intake grip on sample, should be called in deposit after the deposit has a firm grip
                // TODO: check endAffector.inPosition()
                if (intakeTurret.inPosition() && finishTransferRequest) {
                    state = State.TRANSFER_END;
                    finishTransferRequest = false;
                    sampleStatus = false;
                }
                break;
            case TRANSFER_END:
                intakeTurret.setIntakeExtension(0.0);
                intakeTurret.setClawRotation(transferRotation);
                intakeTurret.setTurretArmTarget(turretTransferAngle);
                intakeTurret.setTurretRotation(turretTransferRotation);

                intakeTurret.setClawState(false);

                // once the grab is finished, send back to RETRACT. false grab changes from HOLD to READY
                // no need to worry about whacking stuff b/c both states require rotation to be in the turretTransferRot value
                if (intakeTurret.grabInPosition()) {
                    state = State.RETRACT;
                    grab = false;
                }
                break;

            case TEST:
                intakeTurret.setIntakeExtension(intakeSetTargetPos);
                break;
        }

        intakeTurret.update();
        robot.vision.setNewOrientation(intakeTurret.getTurretRotation());
        updateTelemetry();
    }

    public void useCamera(boolean val) {
        useCamera = val;
    }

    // TODO: Determine minMaxClips for setters
    public void setTurretGrabRot(double t) {
        turretGrabRotation = t;
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
        grabRotation = angle;
    }

    public double getClawRotAngle() {
        return grabRotation;
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
        return intakeTurret.inPosition() && state == State.TRANSFER_WAIT;
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

    public double getIntakeLength() {
        return intakeTurret.intakeExtension.getLength();
    }

    public double getIntakeRelativeToRobot() {
        return getIntakeLength() + IntakeTurret.extendoOffset;
    }

    public double getIntakeTargetPos() {
        return this.intakeSetTargetPos;
    }

//    public void forcePullIn() { forcePull = true; }

    public void updateTelemetry() {
        TelemetryUtil.packet.put("ClawIntake.clawRotationAlignAngle", grabRotation);
        LogUtil.intakeClawRotationAngle.set(grabRotation);
        TelemetryUtil.packet.put("ClawIntake.grab", grab);
        LogUtil.intakeClawGrab.set(grab);
        TelemetryUtil.packet.put("ClawIntake clawRotation angle", intakeTurret.getClawRotation());
        TelemetryUtil.packet.put("ClawIntake State", this.state);
        LogUtil.intakeState.set(this.state.toString());
    }

    public void resetExtendoEncoders() {
        Log.e("RESETTTING", "RESTETING EXTENDO *************");
        intakeTurret.intakeExtension.resetExtendoEncoders();
    }
}
