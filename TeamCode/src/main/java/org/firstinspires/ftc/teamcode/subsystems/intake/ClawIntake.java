package org.firstinspires.ftc.teamcode.subsystems.intake;

import android.util.Log;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.utils.PID;
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

    private double extendoTargetPos;
    private double intakeSetTargetPos;
    private double extendoCurrentPos;

    public static PID extendoPID = new PID(0.15, 0.01, 0.008);
    public static double slidesTolerance = 0.5;
    public static double slidesForcePullPow = -0.3;

    public static double intakeHoverAngle = -1.35;
    public static double intakeFlipConfirmAngle = -1.3;
    public static double intakeFlipUpAngle = -0.7;
    public static double intakeFlipGrabAngle = -1.6;
    public static double intakeFlipBackAngle = -0.1;

    public static double clawRotationDefaultAngle = 0.0;
    private double clawRotationAlignAngle = 0.0;

    public static double clawOpenAngle = 0.2634;
    public static double clawCloseAngle = 1.25;

    private boolean grab = false;

    enum ClawIntakeState {
        START_EXTEND, //ideally this is where limelight is gonna be reading in auto
        FINISH_EXTEND, //flips down while continuing to extend, continuing to read
        ALIGN, //this is where limelight is gonna read for teleop, move to next on buttom press
        LOWER,
        GRAB_CLOSE,
        CONFIRM,
        RETRACT, //flip up + retract
        HOLD,
        READY //ungrip
    }

    private ClawIntakeState clawIntakeState = ClawIntakeState.READY;
    private DcMotorEx m;

    public ClawIntake(Robot robot) {
        this.robot = robot;

        m = robot.hardwareMap.get(DcMotorEx.class, "intakeExtensionMotor");

        intakeExtensionMotor = new PriorityMotor(
                robot.hardwareMap.get(DcMotorEx.class, "intakeExtensionMotor"),
                "intakeExtensionMotor",
                1, 5, robot.sensors
        );
        robot.hardwareQueue.addDevice(intakeExtensionMotor);

        intakeFlipServo = new nPriorityServo(
                new Servo[] {robot.hardwareMap.get(Servo.class, "intakeFlipServo")},
                "intakeFlipServo",
                nPriorityServo.ServoType.HITEC,
                0, 1, 0.69,
                new boolean[] {false},
                1.0, 5.0
        );
        robot.hardwareQueue.addDevice(intakeFlipServo);

        claw = new nPriorityServo(
                new Servo[] {robot.hardwareMap.get(Servo.class, "intakeClaw")},
                "intakeClaw",
                nPriorityServo.ServoType.AXON_MINI,
                0.474, 0.749, 0.47,
                new boolean[] {false},
                1.0, 5
        );
        robot.hardwareQueue.addDevice(claw);

        clawRotation = new nPriorityServo(
                new Servo[] {robot.hardwareMap.get(Servo.class, "intakeClawRotation")},
                "intakeClawRotation",
                nPriorityServo.ServoType.AXON_MINI,
                0.343,0.894,0.629,
                new boolean[] {false},
                1, 5
        );
        robot.hardwareQueue.addDevice(clawRotation);

        //if (Globals.RUNMODE != RunMode.TELEOP) {
            resetExtendoEncoders();
        //}

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
                    this.clawIntakeState = ClawIntakeState.ALIGN;
                    this.grab = false;
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
        }

        TelemetryUtil.packet.put("ClawIntake.clawRotationAlignAngle", clawRotationAlignAngle);
        TelemetryUtil.packet.put("ClawIntake intakeFlipServo", intakeFlipServo.getCurrentAngle());
        TelemetryUtil.packet.put("ClawIntake clawRotation", clawRotation.getCurrentAngle());
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
        if (this.isExtensionAtTarget()) {
            extendoPID.update(0,-1.0,1.0);
            extendoPID.resetIntegral();
            pow = this.extendoTargetPos <= slidesTolerance ? slidesForcePullPow : 0;
        } else {
            pow = extendoPID.update(this.extendoTargetPos - this.extendoCurrentPos, -0.7, 0.7);
        }

        this.intakeExtensionMotor.setTargetPower(pow);

        TelemetryUtil.packet.put("ClawIntake extendo power", pow);
        TelemetryUtil.packet.put("ClawIntake.extendoTargetPos", this.extendoTargetPos);
        TelemetryUtil.packet.put("ClawIntake.extendoCurrentPos", this.extendoCurrentPos);
        TelemetryUtil.packet.put("ClawIntake State", this.clawIntakeState);
    }

    public boolean isExtensionAtTarget() {
        if (this.extendoTargetPos <= slidesTolerance) return this.extendoCurrentPos <= slidesTolerance;
        return Math.abs(this.extendoTargetPos - this.extendoCurrentPos) <= slidesTolerance;
    }

    public void resetExtendoEncoders() {
        Log.e("RESETTTING", "RESTETING EXTENDO *************");

        m.setPower(0);
        m.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        m.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        m.setPower(0);
    }
}
