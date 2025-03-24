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

    public static double intakeStartRot = 0.0, intakeSearchRot = 0.0, intakeGrabRot = 0.0, intakeTransferRot = 0.0;
    public static double turretStartAngle = 0.0, turretPreAngle = Math.toRadians(15), turretSearchAngle = Math.toRadians(40), turretTransferAngle = Math.toRadians(80);
    public static double turretStartRot = 0.0, turretPreRot = Math.toRadians(15), turretSearchRot = Math.toRadians(180), turretTransferRot = 0.0;
    public static double intakeTransferLen = 2.0;

    private boolean grab = false;

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
        TRANSFER,
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
                intakeSetTargetPos = 6.0;
                endAffector.setIntakeExtension(intakeSetTargetPos);
                endAffector.setIntakeRotation(intakeSearchRot);
                endAffector.setTurretAngle(turretPreAngle);
                endAffector.setTurretRot(turretPreRot);

                endAffector.setClawState(grab);

                // Wait for extension past certain length
                if (endAffector.extendoInPosition()) {
                    clawIntakeState = nClawIntakeState.FULL_EXTEND;
                }
                break;
            case FULL_EXTEND:
                // Fully extend + rotate to search positions
                intakeSetTargetPos = 12.0;
                endAffector.setIntakeExtension(intakeSetTargetPos);
                endAffector.setIntakeRotation(intakeStartRot);
                endAffector.setTurretAngle(turretSearchAngle);
                endAffector.setTurretRot(turretSearchRot);

                endAffector.setClawState(grab);

                // Wait for full extension and turret in position before starting search
                if (endAffector.extendoInPosition() && endAffector.turretRotInPosition()) {
                    clawIntakeState = this.grab ? nClawIntakeState.CONFIRM : nClawIntakeState.SEARCH;
                    intakeLight.setState(true);
                }
                break;
            case SEARCH:
                // Begin Serach, just hold positions
                intakeSetTargetPos = 12.0;
                endAffector.setIntakeExtension(intakeSetTargetPos);
                endAffector.setIntakeRotation(intakeSearchRot);
                endAffector.setTurretAngle(turretSearchAngle);
                endAffector.setTurretRot(turretSearchRot);

                endAffector.setClawState(false);

                // Use grab as a flag for whether or not a block has been found
                if (this.grab && endAffector.rotInPosition()) {
                    clawIntakeState = nClawIntakeState.LOWER;
                }
                break;
            case LOWER:
                // intakeAt method should hopefully calculate new extension, new turretAngle + Rotation, and move in to grab
                endAffector.intakeAt(new Pose2d(0, 0, 0));

                endAffector.setClawState(false);

                // everything in position before grabbing
                if (endAffector.inPosition()){
                    clawIntakeState = nClawIntakeState.GRAB_CLOSE;
                }
                break;
            case GRAB_CLOSE:
                // hold prev position
                endAffector.intakeAt(new Pose2d(0, 0, 0));

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
                endAffector.setTurretAngle(turretTransferAngle);
                endAffector.setTurretAngle(turretTransferRot);

                endAffector.setClawState(true);

                // if grab failed go back to search
                if (!this.grab) {
                    clawIntakeState = nClawIntakeState.SEARCH;
                }
                break;
            case RETRACT_BUFFER:
                // begin retraction upon being prompted, have a 6in buffer should be enough for swinging stuff around
                endAffector.setIntakeExtension(intakeTransferLen + 6.0);
                endAffector.setIntakeRotation(intakeTransferRot);
                endAffector.setTurretAngle(turretTransferAngle);
                endAffector.setTurretAngle(turretTransferRot);

                endAffector.setClawState(grab);

                // once turret is in place we can safe retract
                if(endAffector.turretRotInPosition()){
                    clawIntakeState = grab ? nClawIntakeState.HOLD : nClawIntakeState.READY;
                }
                break;
            case RETRACT:
                // full retract into transfer
                endAffector.setIntakeExtension(intakeSetTargetPos);
                endAffector.setIntakeRotation(intakeStartRot);
                endAffector.setTurretAngle(turretStartAngle);
                endAffector.setTurretAngle(turretStartRot);

                endAffector.setClawState(grab);

                if (endAffector.extendoInPosition()) {
                    clawIntakeState = this.grab ? nClawIntakeState.HOLD : nClawIntakeState.READY;
                    this.intakeLight.setState(false);
                }
                break;
            case HOLD:
                // tucked in with sample
                endAffector.setIntakeExtension(0.0);
                endAffector.setIntakeRotation(intakeStartRot);
                endAffector.setTurretAngle(turretStartAngle);
                endAffector.setTurretAngle(turretStartRot);

                endAffector.setClawState(true);
                break;
            case READY:
                // hold in start position, everything tucked in while moving so defense can be played. no sample ver
                endAffector.setIntakeExtension(0.0);
                endAffector.setIntakeExtension(intakeSetTargetPos);
                endAffector.setIntakeRotation(intakeStartRot);
                endAffector.setTurretAngle(turretStartAngle);
                endAffector.setTurretAngle(turretStartRot);

                endAffector.setClawState(false);
                break;
            case TRANSFER:
                // hold in transfer position
                endAffector.setIntakeExtension(intakeTransferLen);
                endAffector.setIntakeRotation(intakeTransferRot);
                endAffector.setTurretAngle(turretTransferAngle);
                endAffector.setTurretAngle(turretTransferRot);

                endAffector.setClawState(true);
                break;
            case TEST:
                endAffector.setIntakeExtension(intakeSetTargetPos);
                break;
        }

        updateTelemetry();
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
