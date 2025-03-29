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
import org.firstinspires.ftc.teamcode.utils.RunMode;
import org.firstinspires.ftc.teamcode.utils.TelemetryUtil;
import org.firstinspires.ftc.teamcode.utils.Utils;
import org.firstinspires.ftc.teamcode.utils.priority.PriorityMotor;
import org.firstinspires.ftc.teamcode.utils.priority.nPriorityServo;

@Config
public class ClawIntake {
    private final Robot robot;

    public PriorityMotor intakeExtensionMotor;
    public nPriorityServo intakeFlipServo;
    public nPriorityServo claw;
    public nPriorityServo clawRotation;

    private DigitalChannel intakeLight;

    private double extendoTargetPos;
    private double intakeSetTargetPos;
    private double extendoCurrentPos;

    public static PID extendoPID = new PID(0.2, 0.05, 0.009);
    public static double slidesTolerance = 0.9;
    public static double slidesDeadZone = 0.2;
    public static double slidesKeepInPow = -0.25;
    public static double slidesForcePullPow = -0.8;
    private boolean forcePull = false;

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

    public enum ClawIntakeState {
        START_EXTEND, //ideally this is where limelight is gonna be reading in auto
        FINISH_EXTEND, //flips down while continuing to extend, continuing to read
        ALIGN, //this is where limelight is gonna read for teleop, move to next on buttom press
        LOWER,
        GRAB_CLOSE,
        CONFIRM,
        RETRACT, //flip up + retract
        HOLD,
        READY, //ungrip
        TEST
    }

    public ClawIntakeState clawIntakeState = ClawIntakeState.READY;
    private DcMotorEx m;

    public ClawIntake(Robot robot) {
        this.robot = robot;

        m = robot.hardwareMap.get(DcMotorEx.class, "intakeExtensionMotor");

        intakeExtensionMotor = new PriorityMotor(
                robot.hardwareMap.get(DcMotorEx.class, "intakeExtensionMotor"),
                "intakeExtensionMotor",
                1, 5, robot.sensors
        );
        //robot.hardwareQueue.addDevice(intakeExtensionMotor);

        intakeFlipServo = new nPriorityServo(
                new Servo[] {robot.hardwareMap.get(Servo.class, "intakeTurretArm")},
                "intakeFlipServo",
                nPriorityServo.ServoType.HITEC,
                0.0, 0.695, 0.69,
                new boolean[] {false},
                1.0, 5.0
        );
        //robot.hardwareQueue.addDevice(intakeFlipServo);

        claw = new nPriorityServo(
                new Servo[] {robot.hardwareMap.get(Servo.class, "intakeClaw")},
                "intakeClaw",
                nPriorityServo.ServoType.AXON_MINI,
                0.46, 0.75, 0.47,
                new boolean[] {false},
                1.0, 5
        );
        //robot.hardwareQueue.addDevice(claw);

        clawRotation = new nPriorityServo(
                new Servo[] {robot.hardwareMap.get(Servo.class, "intakeClawRotation")},
                "intakeClawRotation",
                nPriorityServo.ServoType.AXON_MINI,
                0.06,0.67,0.37,
                new boolean[] {false},
                1, 5
        );
        //robot.hardwareQueue.addDevice(clawRotation);

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
        updateExtendo();

        switch (clawIntakeState) {
            case START_EXTEND: // clawRotation goes up
                this.intakeFlipServo.setTargetAngle(intakeFlipUpAngle);
                this.clawRotation.setTargetAngle(clawRotationDefaultAngle);
                this.claw.setTargetAngle(this.grab ? clawCloseAngle : clawOpenAngle);
                this.extendoTargetPos = 0;
                this.clawRotationAlignAngle = clawRotationDefaultAngle;
                if (this.intakeFlipServo.inPosition()) this.clawIntakeState = ClawIntakeState.FINISH_EXTEND;
                break;
            case FINISH_EXTEND:
                this.intakeFlipServo.setTargetAngle(intakeFlipUpAngle);
                this.clawRotation.setTargetAngle(clawRotationDefaultAngle);
                this.claw.setTargetAngle(this.grab ? clawCloseAngle : clawOpenAngle);
                this.extendoTargetPos = this.intakeSetTargetPos;
                if (this.isExtensionAtTarget()) {
                    this.clawIntakeState = this.grab ? ClawIntakeState.CONFIRM : ClawIntakeState.ALIGN;
                    this.intakeLight.setState(true);
                }
                break;
            case ALIGN:
                this.intakeFlipServo.setTargetAngle(intakeHoverAngle);
                this.clawRotation.setTargetAngle(clawRotationAlignAngle);
                this.claw.setTargetAngle(clawOpenAngle);
                this.extendoTargetPos = this.intakeSetTargetPos;
                if (this.grab && this.clawRotation.inPosition()) this.clawIntakeState = ClawIntakeState.LOWER;
                break;
            case LOWER:
                this.intakeFlipServo.setTargetAngle(intakeFlipGrabAngle);
                this.clawRotation.setTargetAngle(this.clawRotationAlignAngle);
                this.claw.setTargetAngle(clawOpenAngle);
                this.extendoTargetPos = this.intakeSetTargetPos;
                if (this.intakeFlipServo.inPosition())
                    this.clawIntakeState = ClawIntakeState.GRAB_CLOSE;
                break;
            case GRAB_CLOSE:
                this.intakeFlipServo.setTargetAngle(intakeFlipGrabAngle);
                this.clawRotation.setTargetAngle(this.clawRotationAlignAngle);
                this.claw.setTargetAngle(clawCloseAngle);
                this.extendoTargetPos = this.intakeSetTargetPos;
                if (this.claw.inPosition()) this.clawIntakeState = ClawIntakeState.CONFIRM;
                break;
            case CONFIRM:
                this.intakeFlipServo.setTargetAngle(intakeFlipConfirmAngle);
                this.clawRotation.setTargetAngle(this.clawRotationAlignAngle);
                this.claw.setTargetAngle(clawCloseAngle);
                this.extendoTargetPos = this.intakeSetTargetPos;
                if (!this.grab) this.clawIntakeState = ClawIntakeState.ALIGN;
                break;
            case RETRACT:
                this.intakeFlipServo.setTargetAngle(intakeFlipUpAngle);
                this.clawRotation.setTargetAngle(clawRotationDefaultAngle);
                this.claw.setTargetAngle(this.grab ? clawCloseAngle : clawOpenAngle);
                this.extendoTargetPos = 0;
                this.intakeLight.setState(false);
                if (this.isExtensionAtTarget()) {
                    this.intakeFlipServo.setTargetAngle(intakeFlipBackAngle);
                    this.clawIntakeState = this.grab ? ClawIntakeState.HOLD : ClawIntakeState.READY;
                }
                break;
            case HOLD:
                this.intakeFlipServo.setTargetAngle(intakeFlipBackAngle);
                this.clawRotation.setTargetAngle(clawRotationDefaultAngle);
                this.claw.setTargetAngle(clawCloseAngle);
                this.extendoTargetPos = 0;
                break;
            case READY:
                this.intakeFlipServo.setTargetAngle(intakeFlipBackAngle);
                this.clawRotation.setTargetAngle(clawRotationDefaultAngle);
                this.claw.setTargetAngle(clawOpenAngle);
                this.extendoTargetPos = 0;
                break;
            case TEST:
                extendoTargetPos = intakeSetTargetPos;
                break;
        }

        TelemetryUtil.packet.put("ClawIntake.clawRotationAlignAngle", clawRotationAlignAngle);
        LogUtil.intakeClawRotationAngle.set(clawRotationAlignAngle);
        TelemetryUtil.packet.put("ClawIntake.grab", grab);
        LogUtil.intakeClawGrab.set(grab);
        TelemetryUtil.packet.put("ClawIntake intakeFlipServo angle", intakeFlipServo.getCurrentAngle());
        TelemetryUtil.packet.put("ClawIntake clawRotation angle", clawRotation.getCurrentAngle());
        TelemetryUtil.packet.put("ClawIntake State", this.clawIntakeState);
        LogUtil.intakeState.set(this.clawIntakeState.toString());
    }

    public void setClawRotation(double angle) {
        if (angle > 1.7) angle = -1.7;
        else if (angle < -1.7) angle = 1.7;
        this.clawRotationAlignAngle = angle;
    }
    public double getClawRotAngle() {
        return this.clawRotationAlignAngle;
    }

    public void retract() {
        this.clawIntakeState = ClawIntakeState.RETRACT;
    }

    public void grab(boolean closed) {
        grab = closed;
    }

    public boolean grabFinished() {
        return clawIntakeState == ClawIntakeState.CONFIRM;
    }

    public boolean dropFinished() {
        return clawIntakeState == ClawIntakeState.ALIGN;
    }

    public void extend() {
        if (this.clawIntakeState == ClawIntakeState.READY || this.clawIntakeState == ClawIntakeState.HOLD)
            this.clawIntakeState = ClawIntakeState.START_EXTEND;
    }

    public void release() {
        if (this.clawIntakeState == ClawIntakeState.HOLD) {
            this.clawIntakeState = ClawIntakeState.READY;
            grab = false;
        }
    }

    public boolean isRetracted() {
        return (this.clawIntakeState == ClawIntakeState.HOLD || this.clawIntakeState == ClawIntakeState.READY) && this.intakeFlipServo.inPosition();
    }

    public boolean isExtended() {
        return this.clawIntakeState == ClawIntakeState.ALIGN || this.clawIntakeState == ClawIntakeState.LOWER
            || this.clawIntakeState == ClawIntakeState.GRAB_CLOSE || this.clawIntakeState == ClawIntakeState.CONFIRM;
    }

    public boolean hasSample() { return this.grab; }

    public void setIntakeTargetPos(double targetPos) {
        this.intakeSetTargetPos = Utils.minMaxClip(targetPos, 1, 24);
    }
    public double getIntakeTargetPos() {
        return this.intakeSetTargetPos;
    }

    private void updateExtendo() {
        this.extendoCurrentPos = this.robot.sensors.getExtendoPos();

        double pow = 0;

        if (Globals.TESTING_DISABLE_CONTROL && Globals.RUNMODE == RunMode.TESTER) {
            extendoPID.update(0, -1.0, 1.0);
            extendoPID.resetIntegral();
            intakeExtensionMotor.setTargetPower(0.0);
        } else {
            if (this.isExtensionAtTarget(slidesDeadZone)) {
                extendoPID.update(0, -1.0, 1.0);
                extendoPID.resetIntegral();
                pow = this.extendoTargetPos <= slidesTolerance && this.extendoCurrentPos >= 0.0 ? slidesKeepInPow : 0;
            } else {
                pow = extendoPID.update(this.extendoTargetPos - this.extendoCurrentPos, -0.7, 0.7);
            }

            if (forcePull) {
                pow = slidesForcePullPow;
                forcePull = false;
            }

            this.intakeExtensionMotor.setTargetPower(pow);
        }

        TelemetryUtil.packet.put("ClawIntake extendo power", pow);
        TelemetryUtil.packet.put("ClawIntake.extendoTargetPos", this.extendoTargetPos);
        LogUtil.extendoTargetPos.set(this.extendoTargetPos);
        //TelemetryUtil.packet.put("ClawIntake.extendoCurrentPos", this.extendoCurrentPos);
    }

    public void forcePullIn() { forcePull = true; }

    public boolean isExtensionAtTarget() { return this.isExtensionAtTarget(slidesTolerance); }

    private boolean isExtensionAtTarget(double tol) {
        if (this.extendoTargetPos <= tol) return this.extendoCurrentPos <= tol;
        return Math.abs(this.extendoTargetPos - this.extendoCurrentPos) <= tol;
    }

    public void resetExtendoEncoders() {
        Log.e("RESETTTING", "RESTETING EXTENDO *************");

        m.setPower(0);
        m.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        m.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        m.setPower(0);
    }
}
