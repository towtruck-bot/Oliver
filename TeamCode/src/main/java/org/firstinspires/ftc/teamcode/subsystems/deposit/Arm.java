package org.firstinspires.ftc.teamcode.subsystems.deposit;

import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.sensors.Sensors;
import org.firstinspires.ftc.teamcode.utils.priority.nPriorityServo;

public class Arm {
    public final Sensors sensors;

    public final nPriorityServo armRotation;
    public final nPriorityServo clawRotation;
    public final nPriorityServo clawGrip;

    public static double sampleOpenRad = 0.9, sampleCloseRad = 0.55, speciOpenRad = 0.7, speciCloseRad = 0.02;
    public final double armLength = 5.905314961;

    public Arm(Robot robot){
        this.sensors = robot.sensors;

        armRotation = new nPriorityServo(
                new Servo[] {robot.hardwareMap.get(Servo.class, "armRotationL"), robot.hardwareMap.get(Servo.class, "armRotationR")},
                "armRotation",
                nPriorityServo.ServoType.AXON_MINI,
                0,
                0.6,
                0.025,
                new boolean[] {false, true},
                1.0,
                2.0
        );
        robot.hardwareQueue.addDevice(armRotation);

        clawRotation = new nPriorityServo(
                new Servo[] {robot.hardwareMap.get(Servo.class, "clawRotation")},
                "clawRotation",
                nPriorityServo.ServoType.AXON_MINI,
                0.0,
                1.0,
                0.69,
                new boolean[] {true},
                1.0,
                2.0
        );
        robot.hardwareQueue.addDevice(clawRotation);

        clawGrip = new nPriorityServo(
                new Servo[] {robot.hardwareMap.get(Servo.class, "clawGrip")},
                "clawGrip",
                nPriorityServo.ServoType.AXON_MINI,
                0.078,
                0.28,
                0.08,
                new boolean[] {false},
                1.0,
                2.0
        );
        robot.hardwareQueue.addDevice(clawGrip);
    }

    public void setArmRotation(double targetRad, double power){
        armRotation.setTargetAngle(targetRad, power);
    }

    public void setClawRotation(double targetRad, double power){
        clawRotation.setTargetAngle(targetRad, power);
    }

    public void sampleOpen(){
        clawGrip.setTargetAngle(sampleOpenRad, 1.0);
    }

    public void speciOpen(){
        clawGrip.setTargetAngle(speciOpenRad, 1.0);
    }

    public void sampleClose(){
        clawGrip.setTargetAngle(sampleCloseRad, 1.0);
    }

    public void speciClose(){
        clawGrip.setTargetAngle(speciCloseRad, 1.0);
    }

    public boolean inPosition(){
        return armRotation.inPosition() && clawRotation.inPosition();
    }

    public boolean clawFinished(){
        return clawGrip.inPosition();
    }
}
