package org.firstinspires.ftc.teamcode.subsystems.intake;

import android.util.Log;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.utils.Globals;
import org.firstinspires.ftc.teamcode.utils.LogUtil;
import org.firstinspires.ftc.teamcode.utils.PID;
import org.firstinspires.ftc.teamcode.utils.Pose2d;
import org.firstinspires.ftc.teamcode.utils.RunMode;
import org.firstinspires.ftc.teamcode.utils.TelemetryUtil;
import org.firstinspires.ftc.teamcode.utils.Utils;
import org.firstinspires.ftc.teamcode.utils.priority.PriorityMotor;
import org.firstinspires.ftc.teamcode.utils.priority.nPriorityServo;

@Config
public class nClawIntake {
    private final Robot robot;

    private final EndAffector endAffector;

    private final DigitalChannel intakeLight;

    private double intakeSetTargetPos;

    public static double intakeStartAngle, intakeSearchAngle, intakeGrabAngle, intakeTransferAngle;
    public static double intakeStartRot, intakeSearchRot, intakeTransferRot;
    public static double turretStartAngle, turretSearchAngle = Math.toRadians(40), turretTransferAngle;
    public static double turretStartRot, turretSearchRot, turretTransferRot;
    public static double intakeTransferLen;

    public static double intakeHoverAngle = -1.6;
    public static double intakeFlipConfirmAngle = -1.55;
    public static double intakeFlipUpAngle = -0.95;
    public static double intakeFlipGrabAngle = -1.955;
    public static double intakeFlipBackAngle = -0.355;

    public static double clawRotationDefaultAngle = 0.0;
    private double clawRotationAlignAngle = 0.0;

    public static double clawOpenAngle = 0.2634;
    public static double clawCloseAngle = 1.4;

    private boolean grab = false;

    public enum nClawIntakeState {
        START_EXTEND,
        FINISH_EXTEND,
        ALIGN,
        LOWER,
        GRAB_CLOSE,
        CONFIRM,
        RETRACT_BUFFER,
        RETRACT,
        HOLD,
        READY,
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
                endAffector.setIntakeExtension(0.0);
                endAffector.setIntakeAngle(intakeStartAngle);
                endAffector.setIntakeRotation(intakeStartRot);
                endAffector.setTurretAngle(turretStartAngle);
                endAffector.setTurretRot(turretStartRot);
                endAffector.setClawState(grab);

                if (endAffector.flipInPosition()){
                    clawIntakeState = nClawIntakeState.FINISH_EXTEND;
                }
                break;
            case FINISH_EXTEND:
                endAffector.setIntakeExtension(intakeSetTargetPos);
                endAffector.setIntakeAngle(intakeFlipUpAngle);
                endAffector.setIntakeRotation(clawRotationDefaultAngle);
                endAffector.setTurretAngle(turretSearchAngle);
                endAffector.setTurretRot(turretSearchRot);
                endAffector.setClawState(grab);

                if (endAffector.extendoInPosition()) {
                    clawIntakeState = this.grab ? nClawIntakeState.CONFIRM : nClawIntakeState.ALIGN;
                    intakeLight.setState(true);
                }
                break;
            case ALIGN:
                endAffector.setIntakeExtension(intakeSetTargetPos);
                endAffector.setIntakeAngle(intakeSearchAngle);
                endAffector.setIntakeRotation(intakeSearchRot);
                endAffector.setTurretAngle(turretSearchAngle);
                endAffector.setTurretRot(turretSearchRot);

                endAffector.setClawState(false);
                if (this.grab && endAffector.rotInPosition()) {
                    clawIntakeState = nClawIntakeState.LOWER;
                }
                break;
            case LOWER:
                endAffector.intakeAt(new Pose2d(0, 0, 0));
                endAffector.setClawState(false);

                if (endAffector.flipInPosition()){
                    clawIntakeState = nClawIntakeState.GRAB_CLOSE;
                }
                break;
            case GRAB_CLOSE:
                endAffector.intakeAt(new Pose2d(0, 0, 0));
                endAffector.setClawState(true);

                if (endAffector.grabInPosition()) {
                    clawIntakeState = nClawIntakeState.CONFIRM;
                }
                break;
            case CONFIRM:
                endAffector.setIntakeExtension(intakeSetTargetPos);
                endAffector.setIntakeAngle(intakeTransferAngle);
                endAffector.setIntakeRotation(intakeTransferRot);
                endAffector.setTurretAngle(turretTransferAngle);
                endAffector.setTurretAngle(turretTransferRot);
                endAffector.setClawState(true);

                if (!this.grab) {
                    clawIntakeState = nClawIntakeState.ALIGN;
                }
                break;
            case RETRACT_BUFFER:
                endAffector.setIntakeExtension(intakeTransferLen + 4.0);
                endAffector.setIntakeAngle(intakeTransferAngle);
                endAffector.setIntakeRotation(intakeTransferRot);
                endAffector.setTurretAngle(turretTransferAngle);
                endAffector.setTurretAngle(turretTransferRot);
                endAffector.setClawState(grab);

                if(endAffector.turretRotInPosition()){
                    clawIntakeState = grab ? nClawIntakeState.HOLD : nClawIntakeState.READY;
                }
                break;
            case RETRACT:
                endAffector.setIntakeExtension(intakeSetTargetPos);
                endAffector.setIntakeAngle(intakeTransferAngle);
                endAffector.setIntakeRotation(intakeTransferRot);
                endAffector.setTurretAngle(turretTransferAngle);
                endAffector.setTurretAngle(turretTransferRot);
                endAffector.setClawState(grab);

                this.intakeLight.setState(false);

                if (endAffector.extendoInPosition()) {
                    endAffector.setTurretAngle(intakeFlipBackAngle);
                    clawIntakeState = this.grab ? nClawIntakeState.HOLD : nClawIntakeState.READY;
                }
                break;
            case HOLD:
                endAffector.setIntakeExtension(intakeTransferLen);
                endAffector.setIntakeAngle(intakeTransferAngle);
                endAffector.setIntakeRotation(intakeTransferRot);
                endAffector.setTurretAngle(turretTransferAngle);
                endAffector.setTurretAngle(turretTransferRot);
                endAffector.setClawState(true);
                break;
            case READY:
                endAffector.setIntakeExtension(0.0);
                endAffector.setIntakeExtension(intakeSetTargetPos);
                endAffector.setIntakeAngle(intakeStartAngle);
                endAffector.setIntakeRotation(intakeStartRot);
                endAffector.setTurretAngle(turretStartAngle);
                endAffector.setTurretAngle(turretStartRot);
                endAffector.setClawState(false);
                break;
            case TEST:
                endAffector.setIntakeExtension(intakeSetTargetPos);
                break;
        }

        updateTelemetry();
    }

    public void setClawRotation(double angle) {
        if (angle > 1.7) angle = -1.7;
        else if (angle < -1.7) angle = 1.7;
        this.clawRotationAlignAngle = angle;
    }

    public double getClawRotAngle() {
        return this.clawRotationAlignAngle;
    }

    public void extend() {
        if (this.clawIntakeState == nClawIntakeState.READY || this.clawIntakeState == nClawIntakeState.HOLD)
            this.clawIntakeState = nClawIntakeState.START_EXTEND;
    }

    public boolean isExtended() {
        return this.clawIntakeState == nClawIntakeState.ALIGN || this.clawIntakeState == nClawIntakeState.LOWER
                || this.clawIntakeState == nClawIntakeState.GRAB_CLOSE || this.clawIntakeState == nClawIntakeState.CONFIRM;
    }

    public void retract() {
        this.clawIntakeState = nClawIntakeState.RETRACT;
    }

    public boolean isRetracted() {
        return (this.clawIntakeState == nClawIntakeState.HOLD || this.clawIntakeState == nClawIntakeState.READY) && endAffector.flipInPosition();
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
        return clawIntakeState == nClawIntakeState.ALIGN;
    }

    public void setIntakeTargetPos(double targetPos) {
        this.intakeSetTargetPos = Utils.minMaxClip(targetPos, 1, 19);
    }

    public double getIntakeTargetPos() {
        return this.intakeSetTargetPos;
    }

//    public void forcePullIn() { forcePull = true; }

    public void updateTelemetry(){
        TelemetryUtil.packet.put("ClawIntake.clawRotationAlignAngle", clawRotationAlignAngle);
        LogUtil.intakeClawRotationAngle.set(clawRotationAlignAngle);
        TelemetryUtil.packet.put("ClawIntake.grab", grab);
        LogUtil.intakeClawGrab.set(grab);
        TelemetryUtil.packet.put("ClawIntake intakeFlipServo angle", endAffector.getIntakeAngle());
        TelemetryUtil.packet.put("ClawIntake clawRotation angle", endAffector.getIntakeRotation());
        TelemetryUtil.packet.put("ClawIntake State", this.clawIntakeState);
        LogUtil.intakeState.set(this.clawIntakeState.toString());
    }

    public void resetExtendoEncoders() {
        Log.e("RESETTTING", "RESTETING EXTENDO *************");
        endAffector.intakeExtension.resetExtendoEncoders();
    }
}
