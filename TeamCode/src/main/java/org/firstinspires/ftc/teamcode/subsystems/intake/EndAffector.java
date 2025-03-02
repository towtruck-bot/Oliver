package org.firstinspires.ftc.teamcode.subsystems.intake;

import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.utils.Globals;
import org.firstinspires.ftc.teamcode.utils.priority.nPriorityServo;
import org.firstinspires.ftc.teamcode.utils.priority.PriorityMotor;

public class EndAffector {
    private Robot robot;
    public Extendo intakeExtension;
    private nPriorityServo flipServo, clawRotation, claw;

    private double targetRotation = 0.0, targetLength = 0.0;

    private boolean closed;
    public static double clawOpenAngle = 0.2634;
    public static double clawCloseAngle = 1.4;
    public static double extendThresh = 0.05;

    private double targetX, targetY;

    public EndAffector(Robot robot){
        this.robot = robot;

        intakeExtension = new Extendo(robot);

        flipServo = new nPriorityServo(
                new Servo[] {robot.hardwareMap.get(Servo.class, "intakeFlipServo")},
                "flipServo",
                nPriorityServo.ServoType.HITEC,
                0, 0.69, 0.69,
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


        if(Globals.hasSamplePreload){
            targetX = 49.0;
            targetY = 26.0;
        } else{
            targetX = -49.0;
            targetY = 26.0;
        }

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

    public void extendTarget(double flip){
        intakeExtension.setTargetLength(robot.drivetrain.getExtension());
        flipServo.setTargetAngle(flip, 1.0);
        // TODO: Verify this v
        clawRotation.setTargetAngle(3 * Math.PI / 2 - robot.drivetrain.getOptimalHeading());
    }

    public void grab(){
        claw.setTargetAngle(clawCloseAngle, 1.0);
        closed = true;
    }

    public double getClawRot(){
        return clawRotation.getCurrentAngle();
    }

    public double getExtension(){
        return intakeExtension.getLength();
    }

    public void open(){
        claw.setTargetAngle(clawOpenAngle, 1.0);
        closed = false;
    }

    public boolean isClosed(){
        return closed && claw.inPosition();
    }

    public boolean inPosition(){
        return intakeExtension.inPosition() && flipServo.inPosition() && clawRotation.inPosition();
    }

    public boolean extendoInPosition(){
        return intakeExtension.inPosition();
    }

    public boolean flipInPosition(){
        return flipServo.inPosition();
    }

    public boolean rotInPosition(){
        return clawRotation.inPosition();
    }
}
