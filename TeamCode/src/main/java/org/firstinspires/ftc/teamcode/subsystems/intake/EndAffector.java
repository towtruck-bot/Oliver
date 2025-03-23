package org.firstinspires.ftc.teamcode.subsystems.intake;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.utils.Globals;
import org.firstinspires.ftc.teamcode.utils.Pose2d;
import org.firstinspires.ftc.teamcode.utils.priority.PriorityMotor;
import org.firstinspires.ftc.teamcode.utils.priority.nPriorityServo;

import java.util.Locale;

public class EndAffector {
    private final Robot robot;
    public final Extendo intakeExtension;
    private final nPriorityServo clawRotation, claw, turretArm, turretRotation;

    private double targetLength, targetRotation, turretAngle, turretRot;

    private boolean closed;
    public static double clawOpenAngle = 0.2634;
    public static double clawCloseAngle = 1.4;

    public EndAffector(Robot robot){
        this.robot = robot;

        intakeExtension = new Extendo(robot);

        claw = new nPriorityServo(
                new Servo[] {robot.hardwareMap.get(Servo.class, "intakeClaw")},
                "intakeClaw",
                nPriorityServo.ServoType.AXON_MINI,
                0.46, 0.75, 0.47,
                new boolean[] {false},
                1.0, 5
        );
        robot.hardwareQueue.addDevice(claw);

        clawRotation = new nPriorityServo(
                new Servo[] {robot.hardwareMap.get(Servo.class, "intakeClawRotation")},
                "intakeClawRotation",
                nPriorityServo.ServoType.AXON_MINI,
                0.06,0.67,0.37,
                new boolean[] {false},
                1, 5
        );
        robot.hardwareQueue.addDevice(clawRotation);

        turretArm = new nPriorityServo(
                new Servo[] {robot.hardwareMap.get(Servo.class, "turretArm")},
                "turretArm",
                nPriorityServo.ServoType.AXON_MINI,
                0.0, 1.0, 0.5,
                new boolean[] {false},
                1.0, 5
        );
        robot.hardwareQueue.addDevice(turretArm);

        turretRotation = new nPriorityServo(
                new Servo[] {robot.hardwareMap.get(Servo.class, "turretRotation")},
                "turretRotation",
                nPriorityServo.ServoType.AXON_MINI,
                0.0, 1.0, 0.5,
                new boolean[] {false},
                1.0, 5
        );
        robot.hardwareQueue.addDevice(turretRotation);

        claw.setTargetAngle(clawOpenAngle, 1.0);

        targetLength = 0.0;
        targetRotation = 0.0;
        turretAngle = 0.0;
        turretRot = 0.0;

        closed = false;
    }

    public void update() {
        intakeExtension.setTargetLength(targetLength);
        intakeExtension.update();

        clawRotation.setTargetAngle(targetRotation);

        turretArm.setTargetAngle(turretAngle);
        turretRotation.setTargetAngle(turretRot);
    }

    // Target is given in robot centric coordinates
    private double radius = 4.0, turretLength = 5.0;
    public void intakeAt(Pose2d target){
        double xError = target.x;
        double yError = target.y;

        if(Math.abs(yError) < radius){
            targetLength = xError - 9.0 - Math.sqrt(turretLength * turretLength - yError * yError);
            targetRotation = target.heading;
            turretAngle = yError < turretLength ? Math.asin(yError / turretLength) : turretAngle;
            turretRot = Math.atan2(yError, xError - targetLength);
        }
    }

    public void setClawState(boolean closed) {
        this.closed = closed;
        claw.setTargetAngle(this.closed ? clawCloseAngle : clawOpenAngle, 1.0);
    }

    public void setIntakeExtension(double t){
        targetLength = t;
    }

    public void setIntakeRotation(double t){
        targetRotation = t;
    }

    public void setTurretAngle(double t){
        turretAngle = t;
    }

    public void setTurretRot(double t){
        turretRot = t;
    }

    public double getIntakeRotation(){
        return clawRotation.getCurrentAngle();
    }

    public boolean isClosed(){
        return closed && claw.inPosition();
    }

    public boolean inPosition(){
        return intakeExtension.inPosition() && clawRotation.inPosition() && turretArm.inPosition() && turretRotation.inPosition();
    }

    public boolean extendoInPosition(){
        return intakeExtension.inPosition();
    }

    public boolean rotInPosition(){
        return clawRotation.inPosition();
    }

    public boolean grabInPosition(){
        return claw.inPosition();
    }

    public boolean turretRotInPosition(){
        return turretRotation.inPosition();
    }

    public boolean turretAngInPosition(){
        return turretArm.inPosition();
    }
}
