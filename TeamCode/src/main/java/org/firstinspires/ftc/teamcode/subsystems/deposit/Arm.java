package org.firstinspires.ftc.teamcode.subsystems.deposit;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.sensors.Sensors;
import org.firstinspires.ftc.teamcode.utils.priority.nPriorityServo;

@Config
public class Arm {
    public final Sensors sensors;

    public final nPriorityServo armRotation;
    public final nPriorityServo clawRotation;
    public final nPriorityServo clawGrip;

    public static double sampleOpenRad = 0.4, speciOpenRad = 0.25, closeRad = 0.0;

    public Arm(Robot robot){
        this.sensors = robot.sensors;

        armRotation = new nPriorityServo(
                new Servo[] {robot.hardwareMap.get(Servo.class, "armRotationL"), robot.hardwareMap.get(Servo.class, "armRotationR")},
                "armRotation",
                nPriorityServo.ServoType.AXON_MINI,
                0.34,
                1.0,
                0.89,
                new boolean[] {false, true},
                1.0,
                2.0
        );
        robot.hardwareQueue.addDevice(armRotation);

        clawRotation = new nPriorityServo(
                new Servo[] {robot.hardwareMap.get(Servo.class, "clawRotation")},
                "clawRotation",
                nPriorityServo.ServoType.AXON_MAX,
                0,
                0.67,
                0.32,
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
                0.155,
                0.082,
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

    public void clawClose(){
        clawGrip.setTargetAngle(closeRad, 1.0);
    }

    public boolean inPosition(){
        return armRotation.inPosition() && clawRotation.inPosition();
    }

    public boolean clawInPosition(){
        return clawGrip.inPosition();
    }

    public boolean armInPosition() { return armRotation.inPosition(); }
}
