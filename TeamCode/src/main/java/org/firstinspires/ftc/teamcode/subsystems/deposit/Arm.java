package org.firstinspires.ftc.teamcode.subsystems.deposit;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.sensors.Sensors;
import org.firstinspires.ftc.teamcode.utils.Utils;
import org.firstinspires.ftc.teamcode.utils.priority.PriorityServo;
import org.firstinspires.ftc.teamcode.utils.priority.PriorityServoV2;

public class Arm {
    public final Sensors sensors;
    public final PriorityServoV2 horizontalRail;
    public final PriorityServoV2 armRotation;
    public final PriorityServoV2 clawRotation;
    public final PriorityServoV2 clawGrip;

    private final double wheelRadius = 1.0; //TODO: Get value

    public Arm(Robot robot){
        horizontalRail = new PriorityServoV2(
                new Servo[] {robot.hardwareMap.get(Servo.class, "horizontalRail")},
                "horizontalRail",
                PriorityServoV2.ServoType.SPEED,
                1.0,
                new double[] {1.0},
                0.0,
                1.0,
                0.0,
                0.0,
                1.0,
                3.0,
                5.0
        );

        armRotation = new PriorityServoV2(
                new Servo[] {robot.hardwareMap.get(Servo.class, "armRotationL"), robot.hardwareMap.get(Servo.class, "armRotationR")},
                "armRotation",
                PriorityServoV2.ServoType.SPEED,
                1.0,
                new double[] {1.0, -1.0},
                0.0,
                1.0,
                0.0,
                0.0,
                1.0,
                3.0,
                5.0,
        );

        clawRotation = new PriorityServoV2(
                new Servo[] {robot.hardwareMap.get(Servo.class, "clawRotation")},
                "clawRotation",
                PriorityServoV2.ServoType.SPEED,
                1.0,
                new double[] {1.0},
                0.0,
                1.0,
                0.0,
                0.0,
                1.0,
                3.0,
                5.0
        );

        clawGrip = new PriorityServoV2(
                new Servo[] {robot.hardwareMap.get(Servo.class, "clawGrip")},
                "clawGrip",
                PriorityServoV2.ServoType.SPEED,
                1.0,
                new double[] {1.0},
                0.0,
                1.0,
                0.0,
                0.0,
                1.0,
                3.0,
                5.0
        );

        this.sensors = robot.sensors;
    }

    public void setHorizontalPos(double target){
        horizontalRail.setTargetAngle()
    }
}
