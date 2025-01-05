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

    private double slidesTargetPos;
    private double slidesCurrentPos;

    public static PID extendoPID = new PID(0.185, 0.002, 0.008);
    public static double slidesTolerance = 1;

    public static double intakeClawClose = 0;
    public static double intakeClawOpen = 1; //todo get these values

    enum clawIntakeState {
        START_EXTEND, //ideally this is where limelight is gonna be reading in auto
        FINISH_EXTEND, //flips down while continuing to extend, continuing to read
        EXTENDED, //this is where limelight is gonna read for teleop, move to next on buttom press
        ADJUST, //where limelight adjustments iimplemented
        GRAB, //yoink and immediate retract upon button,
        RETRACT, //flip up + retract
        READY //ungrip
    }

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
                0, 1, 0,
                new boolean[] {false},
                1.0, 5
        );
        robot.hardwareQueue.addDevice(claw);

        clawRotation = new nPriorityServo(
                new Servo[] {robot.hardwareMap.get(Servo.class, "intakeClawRotation")},
                "intakeClawRotation",
                nPriorityServo.ServoType.AXON_MINI,
                0,1,0,
                new boolean[] {false},
                1, 5
        );
        robot.hardwareQueue.addDevice(clawRotation);
    }
    //set slide position
    public void setSlidesTargetPos(double targetPos) {
        this.slidesTargetPos = targetPos;
    }
    //get slide position
    public double getExtendoPos() {
        return slidesCurrentPos;
    }

    //general update for entire class
    public void update() {
        updateSlides();
    }
    //update the slides alone, to be run every loop
    private void updateSlides() {
        if (isExtensionAtTarget()) {
            extendoPID.update(0,-1.0,1.0);
            extendoPID.resetIntegral();
            intakeExtensionMotor.setTargetPower(0.0);
        } else {
            intakeExtensionMotor.setTargetPower(extendoPID.update(slidesTargetPos - slidesCurrentPos, -1.0, 1.0));
        }
    }


    public boolean isExtensionAtTarget() { return Math.abs(slidesTargetPos - slidesCurrentPos) <= slidesTolerance; }
}
