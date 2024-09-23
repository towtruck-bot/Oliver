package org.firstinspires.ftc.teamcode.subsystems.deposit;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.sensors.Sensors;
import org.firstinspires.ftc.teamcode.utils.priority.PriorityServo;

public class Arm {
    public final Sensors sensors;
    public final PriorityServo mgnLinkage;
    public final PriorityServo armRotation;
    public final PriorityServo diffy;
    public final PriorityServo clawActuation;

    public Arm(Robot robot){
        Servo[] mgn = new Servo[] {hardwareMap.get(Servo.class, "mgnServoL"), hardwareMap.get(Servo.class,"mgnServoR")};
        mgnLinkage = new PriorityServo(
                mgn,
                "mgnLinkage",
                PriorityServo.ServoType.SPEED,
                1.0,
                0.0,
                1.0,
                0.0,
                false,
                3.0,
                5.0,
                new double[] {-1.0, 1.0}
        );

        Servo[] arm = new Servo[] {hardwareMap.get(Servo.class, "armServoL"), hardwareMap.get(Servo.class, "armServoR")};
        armRotation = new PriorityServo(
                mgn,
                "armRotation",
                PriorityServo.ServoType.SPEED,
                1.0,
                0.0,
                1.0,
                0.0,
                false,
                3.0,
                5.0,
                new double[] {-1.0, 1.0}
        );

        Servo[] diff = new Servo[] {hardwareMap.get(Servo.class, "diffL"), hardwareMap.get(Servo.class, "diffR")};
        diffy = new PriorityServo(
                mgn,
                "diffy",
                PriorityServo.ServoType.SPEED,
                1.0,
                0.0,
                1.0,
                0.0,
                false,
                3.0,
                5.0,
                new double[] {-1.0, 1.0}
        );

        Servo[] claw = new Servo[] {hardwareMap.get(Servo.class, "claw")};
        clawActuation = new PriorityServo(
                mgn,
                "clawActuation",
                PriorityServo.ServoType.SPEED,
                1.0,
                0.0,
                1.0,
                0.0,
                false,
                3.0,
                5.0,
                new double[] {-1.0, 1.0}
        );

        this.sensors = robot.sensors;
    }

    public boolean checkReady(){
        return mgnLinkage.inPosition() && armRotation.inPosition() && diffy.inPosition() && clawActuation.inPosition();
    }
}
