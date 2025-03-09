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

    public static double sampleOpenRad = 0.22, speciOpenRad = 0.11, closeRad = -0.0106;

    public Arm(Robot robot){
        this.sensors = robot.sensors;

        armRotation = new nPriorityServo(
                new Servo[] {robot.hardwareMap.get(Servo.class, "armRotationL"), robot.hardwareMap.get(Servo.class, "armRotationR")},
                "armRotation",
                nPriorityServo.ServoType.AXON_MINI,
                0,
                0.72,
                0.11,
                new boolean[] {false, true},
                1.0,
                2.0
        );
        robot.hardwareQueue.addDevice(armRotation);

        clawRotation = new nPriorityServo(
                new Servo[] {robot.hardwareMap.get(Servo.class, "clawRotation")},
                "clawRotation",
                nPriorityServo.ServoType.AXON_MINI,
                0,
                0.7,
                0.325,
                new boolean[] {true},
                1.0,
                2.0
        );
        robot.hardwareQueue.addDevice(clawRotation);

        clawGrip = new nPriorityServo(
                new Servo[] {robot.hardwareMap.get(Servo.class, "clawGrip")},
                "clawGrip",
                nPriorityServo.ServoType.AXON_MINI,
                0.076,
                0.27,
                0.08,
                new boolean[] {false},
                1.0,
                2.0
        );
        robot.hardwareQueue.addDevice(clawGrip);

        armRotation.setTargetAngle(0.02, 1);
        clawRotation.setTargetAngle(0.02, 1);
        clawGrip.setTargetAngle(0.02, 1);
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

    public boolean clawFinished(){
        return clawGrip.inPosition();
    }
}
