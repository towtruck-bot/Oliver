package org.firstinspires.ftc.teamcode.subsystems.intake;

import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.utils.Globals;
import org.firstinspires.ftc.teamcode.utils.priority.nPriorityServo;

public class EndAffector {
    private final Robot robot;
    public final Extendo intakeExtension;
    private final nPriorityServo flipServo, clawRotation, claw;

    private double targetRotation, targetLength;

    private boolean closed;
    public static double clawOpenAngle = 0.2634;
    public static double clawCloseAngle = 1.4;

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


        if (Globals.hasSamplePreload) {
            targetX = 49.0;
            targetY = 26.0;
        } else {
            targetX = -49.0;
            targetY = 26.0;
        }

        closed = false;
    }

    public void update() {
        intakeExtension.update();
    }

    public void extend(double extension, double flip, double rotation) {
        intakeExtension.setTargetLength(extension);
        flipServo.setTargetAngle(flip, 1.0);
        clawRotation.setTargetAngle(rotation, 1.0);
    }

	public void setClawState(boolean closed) {
		this.closed = closed;
		this.claw.setTargetAngle(this.closed ? clawCloseAngle : clawOpenAngle, 1.0);
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
