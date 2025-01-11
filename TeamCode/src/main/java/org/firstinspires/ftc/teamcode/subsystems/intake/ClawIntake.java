package org.firstinspires.ftc.teamcode.subsystems.intake;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.sensors.Sensors;
import org.firstinspires.ftc.teamcode.utils.PID;
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

    private double intakeFlipDownAngle = 0.0;
    private double intakeFlipUpAngle = 0.0;
    private double intakeFlipGrabAngle = 0.0;

    private double clawRotationDefaultAngle = 0.0;
    private double clawRotationAdjustAngle = 0.0;

    private double clawOpenAngle = 0.0;
    private double clawCloseAngle = 0.0;

    public static double intakeClawClose = 0;
    public static double intakeClawOpen = 1; //todo get these values
    public static double flipGearRatio = -24.0 / 40.0;

    enum ClawIntakeState {
        START_EXTEND, //ideally this is where limelight is gonna be reading in auto
        FINISH_EXTEND, //flips down while continuing to extend, continuing to read
        EXTENDED, //this is where limelight is gonna read for teleop, move to next on buttom press
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
                0.15, 0.7, 0.7,
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
        switch (clawIntakeState) {
            case START_EXTEND: // clawRotation goes up
                intakeFlipServo.setTargetAngle(intakeFlipDownAngle * flipGearRatio);
                if (clawRotation.getCurrentAngle() < -Math.toRadians(60)) { // TODO: modify angle to min angle claw rotation needs to go out before we can start extendo
                    clawIntakeState = ClawIntakeState.FINISH_EXTEND;
                }
                break;
            case FINISH_EXTEND:
                setExtendoTargetPos(10);
                if (isExtensionAtTarget()) {
                    clawIntakeState = ClawIntakeState.EXTENDED;
                }
                break;
            case EXTENDED:
                clawRotation.setTargetAngle(clawRotationAdjustAngle);
                intakeFlipServo.setTargetAngle(intakeFlipGrabAngle); //grabbing is done through method

                if (retract) {
                    retract = false;
                    clawIntakeState = ClawIntakeState.RETRACT;
                }
                break;
            case RETRACT:
                intakeFlipServo.setTargetAngle(intakeFlipUpAngle);
                setExtendoTargetPos(0);
                break;
            case READY:
                clawRotation.setTargetAngle(clawRotationDefaultAngle);
                intakeFlipServo.setTargetAngle(intakeFlipUpAngle);
                claw.setTargetAngle(clawOpenAngle);
                break;
        }

        updateExtendo();
    }

    public void updateClawRotationAdjustAngle(double clawRotationAdjustAngle) {
        this.clawRotationAdjustAngle = clawRotationAdjustAngle;
    }

    boolean retract = false;
    public void retract() {
        retract = true;
    }

    public void grab() {
        claw.setTargetAngle(clawCloseAngle);
    }

    public void ungrab() {
        claw.setTargetAngle(clawOpenAngle);
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
    }

    public boolean isExtensionAtTarget() { return Math.abs(extendoTargetPos - extendoCurrentPos) <= slidesTolerance; }
}
