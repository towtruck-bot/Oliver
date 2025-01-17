package org.firstinspires.ftc.teamcode.subsystems.intake;

import static org.firstinspires.ftc.teamcode.utils.Globals.GET_LOOP_TIME;

import android.util.Log;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.sensors.Sensors;
import org.firstinspires.ftc.teamcode.utils.PID;
import org.firstinspires.ftc.teamcode.utils.TelemetryUtil;
import org.firstinspires.ftc.teamcode.utils.priority.HardwareQueue;
import org.firstinspires.ftc.teamcode.utils.priority.PriorityMotor;
import org.firstinspires.ftc.teamcode.utils.priority.nPriorityServo;

public class ClawIntake {
    Robot robot;
    Sensors sensors;
    HardwareQueue hardwareQueue;

    PriorityMotor intakeExtensionMotor;
    nPriorityServo intakeFlipServo;
    nPriorityServo claw;
    nPriorityServo clawRotation;

    private double extendoTargetPos;
    private double extendoCurrentPos;

    public static PID extendoPID = new PID(0.185, 0.002, 0.008);
    public static double slidesTolerance = 1;

    private double intakeFlipUpAngle = -0.4416;
    private double intakeFlipGrabAngle = 0.0;
    private double intakeFlipMiddleAngle = -1.5169;

    private double clawRotationDefaultAngle = 0.0;
    private double clawRotationAlignAngle = 0.0;

    private double clawOpenAngle = 0.0;
    private double clawCloseAngle = 0.0;

    public static double intakeClawClose = 0;
    public static double intakeClawOpen = 1; //todo get these values
    public static double flipGearRatio = -24.0 / 40.0;

    enum ClawIntakeState {
        START_EXTEND, //ideally this is where limelight is gonna be reading in auto
        FINISH_EXTEND, //flips down while continuing to extend, continuing to read
        ALIGN, //this is where limelight is gonna read for teleop, move to next on buttom press
        GRAB,
        CONFIRM,
        RETRACT, //flip up + retract
        READY //ungrip
    }

    private ClawIntakeState clawIntakeState = ClawIntakeState.READY;

    public ClawIntake(Robot robot) {
        this.robot = robot;
        this.sensors = robot.sensors;

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
                0.343,0.894,0.621,
                new boolean[] {false},
                1, 5
        );
        robot.hardwareQueue.addDevice(clawRotation);
    }
    //set slide position
    public void setExtendoTargetPos(double targetPos) {
        this.extendoTargetPos = targetPos;
    }
    //get slide position
    public double getExtendoPos() {
        return extendoCurrentPos;
    }

    //general update for entire class
    public void update() {
        extendoCurrentPos = this.robot.sensors.getIntakeExtensionPosition();

        switch (clawIntakeState) {
            case START_EXTEND: // clawRotation goes up
                intakeFlipServo.setTargetAngle(intakeFlipMiddleAngle);
                if (clawRotation.getCurrentAngle() > -1) { // TODO: modify angle to min angle claw rotation needs to go out before we can start extendo
                    clawIntakeState = ClawIntakeState.FINISH_EXTEND;
                    setExtendoTargetPos(10);
                }
                break;
            case FINISH_EXTEND:
                if (isExtensionAtTarget()) {
                    clawIntakeState = ClawIntakeState.ALIGN;
                }
                break;
            case ALIGN:
                clawRotation.setTargetAngle(clawRotationAlignAngle);
                if (grab) {
                    clawIntakeState = ClawIntakeState.GRAB;
                }
                break;
            case GRAB:
                intakeFlipServo.setTargetAngle(intakeFlipGrabAngle);
                clawRotation.setTargetAngle(clawRotationAlignAngle);
                break;
            case CONFIRM:
                clawRotation.setTargetAngle(clawRotationDefaultAngle);
                if (retract) {
                    retract = false;
                    clawIntakeState = ClawIntakeState.RETRACT;
                }
                if (grab) {
                    clawIntakeState = ClawIntakeState.GRAB;
                    grab = false;
                }
                break;
            case RETRACT:
                intakeFlipServo.setTargetAngle(intakeFlipUpAngle);
                setExtendoTargetPos(0);
                break;
            case READY:
                clawRotation.setTargetAngle(clawRotationDefaultAngle);
                intakeFlipServo.setTargetAngle(intakeFlipUpAngle);
                Log.i("SLCI", "" + intakeFlipServo.getTargetAngle());
                claw.setTargetAngle(clawOpenAngle);
                break;
        }

        updateExtendo();
    }

    public void updateClawRotationAdjustAngle(double clawRotationAlignAngle) {
        this.clawRotationAlignAngle = clawRotationAlignAngle;
    }

    boolean retract = false;
    public void retract() {
        retract = true;
    }

    boolean grab = false;
    public void grab() {
        grab = true;
        claw.setTargetAngle(clawCloseAngle);
    }

    public void extend(){
        clawIntakeState = ClawIntakeState.START_EXTEND;
    }

    public boolean isRetracted() {
        return this.clawIntakeState == ClawIntakeState.RETRACT && isExtensionAtTarget();
    }

    //update the slides alone, to be run every loop
    private void updateExtendo() {
        if (isExtensionAtTarget()) {
            extendoPID.update(0,-1.0,1.0);
            extendoPID.resetIntegral();
            intakeExtensionMotor.setTargetPower(0.0);
        } else {
            intakeExtensionMotor.setTargetPower(extendoPID.update(extendoTargetPos - extendoCurrentPos, -1.0, 1.0));
        }

        TelemetryUtil.packet.put("ClawIntake.extendoCurrentPos", extendoCurrentPos);
        TelemetryUtil.packet.put("Claw Intake State", clawIntakeState);
        TelemetryUtil.sendTelemetry();
    }

    public boolean isExtensionAtTarget() { return Math.abs(extendoTargetPos - extendoCurrentPos) <= slidesTolerance; }

    public void rotateClaw(double target){
        clawRotation.setTargetAngle(target);
    }

    public double getClawRotation(){
        return clawRotation.getCurrentAngle();
    }
}
