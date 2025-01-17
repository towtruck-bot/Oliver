package org.firstinspires.ftc.teamcode.subsystems.intake;

import android.util.Log;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.sensors.Sensors;
import org.firstinspires.ftc.teamcode.utils.Globals;
import org.firstinspires.ftc.teamcode.utils.PID;
import org.firstinspires.ftc.teamcode.utils.RunMode;
import org.firstinspires.ftc.teamcode.utils.TelemetryUtil;
import org.firstinspires.ftc.teamcode.utils.priority.HardwareQueue;
import org.firstinspires.ftc.teamcode.utils.priority.PriorityMotor;
import org.firstinspires.ftc.teamcode.utils.priority.nPriorityServo;

@Config
public class ClawIntake {
    Robot robot;
    Sensors sensors;
    HardwareQueue hardwareQueue;

    PriorityMotor intakeExtensionMotor;
    nPriorityServo intakeFlipServo;
    nPriorityServo claw;
    nPriorityServo clawRotation;

    private double extendoTargetPos;
    private double intakeSetTargetPos;
    private double extendoCurrentPos;

    public static PID extendoPID = new PID(0.135, 0.002, 0.005);
    public static double slidesTolerance = 0.5;

    public static double intakeFlipUpAngle = -0.4416;
    public static double intakeFlipGrabAngle = 0.0;
    public static double intakeFlipMiddleAngle = -1.5169;

    public static double clawRotationDefaultAngle = 0.0;
    public static double clawRotationAlignAngle = 0.0;

    public static double clawOpenAngle = 0.2634;
    public static double clawCloseAngle = 0.9918;

    private boolean grab = false;

    enum ClawIntakeState {
        START_EXTEND, //ideally this is where limelight is gonna be reading in auto
        FINISH_EXTEND, //flips down while continuing to extend, continuing to read
        ALIGN, //this is where limelight is gonna read for teleop, move to next on buttom press
        GRAB_MOVE,
        CONFIRM,
        RETRACT, //flip up + retract
        HOLD,
        READY //ungrip
    }

    private ClawIntakeState clawIntakeState = ClawIntakeState.READY;
    private DcMotorEx m;

    public ClawIntake(Robot robot) {
        this.robot = robot;
        this.sensors = robot.sensors;

        m = robot.hardwareMap.get(DcMotorEx.class, "intakeExtensionMotor");

        //innit motors and servos
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
                0.229, 0.692, 0.69,
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

        if (Globals.RUNMODE != RunMode.TELEOP) {
            resetExtendoEncoders();
        }
    }

    //general update for entire class
    public void update() {
        switch (clawIntakeState) {
            case START_EXTEND: // clawRotation goes up
                this.intakeFlipServo.setTargetAngle(intakeFlipMiddleAngle);
                this.clawRotation.setTargetAngle(clawRotationDefaultAngle);
                this.claw.setTargetAngle(clawOpenAngle);
                this.extendoTargetPos = 0;
                this.intakeSetTargetPos = 15;
                if (this.intakeFlipServo.inPosition()) this.clawIntakeState = ClawIntakeState.FINISH_EXTEND;
                // TODO: modify angle to min angle claw rotation needs to go out before we can start extendo
                break;
            case FINISH_EXTEND:
                this.intakeFlipServo.setTargetAngle(intakeFlipMiddleAngle);
                this.clawRotation.setTargetAngle(clawRotationDefaultAngle);
                this.claw.setTargetAngle(clawOpenAngle);
                this.extendoTargetPos = this.intakeSetTargetPos;
                if (this.isExtensionAtTarget()) {
                    this.clawIntakeState = ClawIntakeState.ALIGN;
                    this.grab = false;
                }
                break;
            case ALIGN:
                this.intakeFlipServo.setTargetAngle(intakeFlipUpAngle);
                this.clawRotation.setTargetAngle(clawRotationAlignAngle);
                this.claw.setTargetAngle(clawOpenAngle);
                this.extendoTargetPos = this.intakeSetTargetPos;
                if (this.grab && this.clawRotation.inPosition()) this.clawIntakeState = ClawIntakeState.GRAB_MOVE;
                break;
            case GRAB_MOVE:
                this.intakeFlipServo.setTargetAngle(intakeFlipGrabAngle);
                this.clawRotation.setTargetAngle(clawRotationAlignAngle);
                this.claw.setTargetAngle(clawOpenAngle);
                this.extendoTargetPos = this.intakeSetTargetPos;
                if (this.intakeFlipServo.inPosition()) this.clawIntakeState = ClawIntakeState.CONFIRM;
                break;
            case CONFIRM:
                this.intakeFlipServo.setTargetAngle(intakeFlipGrabAngle);
                this.clawRotation.setTargetAngle(clawRotationAlignAngle);
                this.claw.setTargetAngle(clawCloseAngle);
                this.extendoTargetPos = this.intakeSetTargetPos;
                if (!this.grab) this.clawIntakeState = ClawIntakeState.ALIGN;
                break;
            case RETRACT:
                this.intakeFlipServo.setTargetAngle(intakeFlipUpAngle);
                this.clawRotation.setTargetAngle(clawRotationDefaultAngle);
                this.claw.setTargetAngle(clawCloseAngle);
                this.extendoTargetPos = 0;
                if (this.isExtensionAtTarget()) this.clawIntakeState = ClawIntakeState.HOLD;
                break;
            case HOLD:
                this.intakeFlipServo.setTargetAngle(intakeFlipMiddleAngle);
                this.clawRotation.setTargetAngle(clawRotationDefaultAngle);
                this.claw.setTargetAngle(clawCloseAngle);
                this.extendoTargetPos = 0;
                break;
            case READY:
                this.intakeFlipServo.setTargetAngle(intakeFlipMiddleAngle);
                this.clawRotation.setTargetAngle(clawRotationDefaultAngle);
                this.claw.setTargetAngle(clawOpenAngle);
                this.extendoTargetPos = 0;
                break;
        }

        updateExtendo();
    }

    public void setClawRotation(double angle) {
        clawRotationAlignAngle = angle;
    }
    public double getClawRotAngle() {
        return clawRotationAlignAngle;
    }

    public void retract() {
        this.clawIntakeState = ClawIntakeState.RETRACT;
    }

    public void grab(boolean closed) {
        grab = closed;
    }

    public void extend() {
        clawIntakeState = ClawIntakeState.START_EXTEND;
    }

    public void release() {
        clawIntakeState = ClawIntakeState.READY;
    }

    public boolean isRetracted() {
        return this.clawIntakeState == ClawIntakeState.HOLD;
    }

    public boolean hasSample() { return this.grab; }

    public void setIntakeTargetPos(double targetPos) {
        this.intakeSetTargetPos = targetPos;
    }
    public double getIntakeTargetPos() {
        return this.intakeSetTargetPos;
    }

    private void updateExtendo() {
        extendoCurrentPos = this.robot.sensors.getExtendoPosition();
        double pow = extendoPID.update(extendoTargetPos - extendoCurrentPos, -1.0, 1.0);

        intakeExtensionMotor.setTargetPower(pow);

        TelemetryUtil.packet.put("ClawIntake slides power", pow);
        TelemetryUtil.packet.put("ClawIntake.extendoTargetPos", extendoTargetPos);
        TelemetryUtil.packet.put("ClawIntake.extendoCurrentPos", extendoCurrentPos);
        TelemetryUtil.packet.put("ClawIntake State", clawIntakeState);
    }

    public boolean isExtensionAtTarget() { return Math.abs(extendoTargetPos - extendoCurrentPos) <= slidesTolerance; }

    private void resetExtendoEncoders() {
        Log.e("RESETTTING", "RESTETING EXTENDO *************");

        m.setPower(0);
        m.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        m.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        m.setPower(0);
    }
}
