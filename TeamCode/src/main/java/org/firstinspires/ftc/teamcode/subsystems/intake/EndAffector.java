package org.firstinspires.ftc.teamcode.subsystems.intake;

import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.utils.priority.nPriorityServo;
import org.firstinspires.ftc.teamcode.utils.priority.PriorityMotor;

public class EndAffector {
    private Robot robot;
    private Extendo intakeExtension;
    private nPriorityServo flipServo, clawRotation, claw;

    private double targetRotation = 0.0, targetLength = 0.0;

    private boolean closed;
    public static double clawOpenAngle = 0.2634;
    public static double clawCloseAngle = 1.4;
    public static double extendThresh = 0.05;

    public EndAffector(Robot robot){
        this.robot = robot;

        intakeExtension = new Extendo(robot);

        flipServo = new nPriorityServo(
                new Servo[] {robot.hardwareMap.get(Servo.class, "intakeFlipServo")},
                "flipServo",
                nPriorityServo.ServoType.HITEC,
                0, 1, 0.69,
                new boolean[] {false},
                1.0, 5.0
        );
        robot.hardwareQueue.addDevice(flipServo);

        clawRotation = new nPriorityServo(
                new Servo[] {robot.hardwareMap.get(Servo.class, "intakeClawRotation")},
                "clawRotation",
                nPriorityServo.ServoType.AXON_MINI,
                0.343,0.894,0.629,
                new boolean[] {false},
                1, 5
        );
        robot.hardwareQueue.addDevice(clawRotation);

        claw = new nPriorityServo(
                new Servo[] {robot.hardwareMap.get(Servo.class, "intakeClaw")},
                "intakeClaw",
                nPriorityServo.ServoType.AXON_MINI,
                0.474, 0.749, 0.47,
                new boolean[] {false},
                1.0, 5
        );
        robot.hardwareQueue.addDevice(claw);

        claw.setTargetAngle(clawOpenAngle, 1.0);

        targetRotation = 0.0;
        targetLength = 0.0;
        closed = false;
    }

    public void update(){
        intakeExtension.update();
    }

    public void extendManual(double extension, double flip, double rotation){
        intakeExtension.setTargetLength(extension);
        flipServo.setTargetAngle(flip, 1.0);
        clawRotation.setTargetAngle(rotation, 1.0);
    }

    public void grab(){
        claw.setTargetAngle(clawCloseAngle, 1.0);
        closed = true;
    }

    public void open(){
        claw.setTargetAngle(clawOpenAngle, 1.0);
        closed = false;
    }

    public boolean isClosed(){
        return closed && claw.inPosition();
    }

    public boolean canExtend(double currX, double currY, double currH, double targetX, double targetY){
        double dx = targetX - currX;
        double dy = targetY - currY;
        return Math.abs(Math.atan2(dy, dx)) < extendThresh;
    }

    public boolean inPosition(){
        return intakeExtension.inPosition(0.5) && flipServo.inPosition() && clawRotation.inPosition();
    }

    public boolean extendoInPosition(double thresh){
        return intakeExtension.inPosition(thresh);
    }

    public boolean flipInPosition(){
        return flipServo.inPosition();
    }

    public boolean rotInPosition(){
        return clawRotation.inPosition();
    }
}
