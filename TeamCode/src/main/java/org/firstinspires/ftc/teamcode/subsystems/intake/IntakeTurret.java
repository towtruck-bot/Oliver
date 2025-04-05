package org.firstinspires.ftc.teamcode.subsystems.intake;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.utils.AngleUtil;
import org.firstinspires.ftc.teamcode.utils.Pose2d;
import org.firstinspires.ftc.teamcode.utils.Vector2;
import org.firstinspires.ftc.teamcode.utils.priority.nPriorityServo;

@Config
public class IntakeTurret {
    private final Robot robot;
    public final IntakeExtension intakeExtension;
    private final nPriorityServo clawRotation, claw, turretArm, turretRotation;

    private double targetLength, clawRotationTarget, turretArmTarget, turretRotationTarget;

    private boolean closed;
    public static double clawOpenAngle = 0.0958;
    public static double clawCloseAngle = 1.219;

    public IntakeTurret(Robot robot){
        this.robot = robot;

        intakeExtension = new IntakeExtension(robot);

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
                new Servo[] {robot.hardwareMap.get(Servo.class, "intakeTurretArm")},
                "intakeTurretArm",
                nPriorityServo.ServoType.HITEC,
                0, 1, 0.1118,
                new boolean[] {false},
                1.0, 5
        );
        robot.hardwareQueue.addDevice(turretArm);

        turretRotation = new nPriorityServo(
                new Servo[] {robot.hardwareMap.get(Servo.class, "intakeTurretRotation")},
                "intakeTurretRotation",
                nPriorityServo.ServoType.AXON_MINI,
                0.07, 1.0, 0.07,
                new boolean[] {false},
                1.0, 5
        );
        robot.hardwareQueue.addDevice(turretRotation);

        claw.setTargetAngle(clawOpenAngle, 1.0);

        targetLength = 0.0;
        clawRotationTarget = 0.0;
        turretArmTarget = 0.0;
        turretRotationTarget = 0.0;

        closed = false;
    }

    public void update() {
        intakeExtension.setTargetLength(targetLength);
        intakeExtension.update();
        //clawRotation.setTargetAngle(clawRotationTarget);

        //turretArm.setTargetAngle(turretArmTarget);
        //turretRotation.setTargetAngle(turretRotationTarget * (48.0 / 40));
    }

    // Target is given in robot centric coordinates
    public static double turretLengthTip = 6;
    public static double turretLengthLL = 4.4;
    public static double extendoOffset = 3.5;
    public static double stupidConstant = -0.25;
    public void intakeAt(Pose2d target){
        double xError = target.x;
        double yError = target.y;

        if(Math.abs(yError) < turretLengthTip){
            targetLength = xError - extendoOffset - Math.sqrt(turretLengthTip * turretLengthTip - yError * yError)/* - yError / turretLength*/;
            setTurretRotation(Math.PI + Math.atan2(yError, Math.sqrt(turretLengthTip * turretLengthTip - yError * yError)));
            setClawRotation(AngleUtil.mirroredClipAngleTolerence(target.heading - getTurretRotation(), Math.toRadians(20)));
            setTurretArmTarget(nClawIntake.turretGrabAngle);
        }
    }

    public void extendTo(Vector2 target){
        double xError = target.x;
        double yError = target.y;

        if(Math.abs(yError) < turretLengthLL){
            targetLength = xError - extendoOffset - Math.sqrt(turretLengthLL * turretLengthLL - yError * yError);
            setTurretRotation(Math.PI + Math.atan2(yError, Math.sqrt(turretLengthLL * turretLengthLL - yError * yError)));
        }
    }

    public void setClawState(boolean closed) {
        this.closed = closed;
        claw.setTargetAngle(this.closed ? clawCloseAngle : clawOpenAngle, 1.0);
    }

    public void setIntakeExtension(double t){
        targetLength = t;
    }

    public void setClawRotation(double t){
        clawRotation.setTargetAngle(t);
        //clawRotationTarget = t;
    }

    public void setTurretArmTarget(double t){
        turretArm.setTargetAngle(t);
        //turretArmTarget = t;
    }

    public void setTurretRotation(double t){
        turretRotation.setTargetAngle(t * (3.524 / Math.PI));
        //turretRotationTarget = t;
    }

    public double getTurretRotation() {
        return turretRotation.getCurrentAngle() / (3.524 / Math.PI);
    }

    public double getClawRotation(){
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

    public boolean clawInPosition() {
        return claw.inPosition();
    }
}
