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
    private final double smallGearNum = 20.0, smallGearRad = 1.0, bigGearNum = 40.0, bigGearRad = 1.0;
    public final double armLength = 1.0; //TODO: Get value
    private double currentHoriPos = 0.0;

    public Arm(Robot robot){
        horizontalRail = new PriorityServoV2(
                new Servo[] {robot.hardwareMap.get(Servo.class, "horizontalRail")},
                "horizontalRail",
                PriorityServoV2.ServoType.SPEED,
                1.0,
                new boolean[] {false},
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
                new boolean[] {false, true}, //TODO: Check these - James
                0.0,
                1.0,
                0.0,
                0.0,
                1.0,
                3.0,
                5.0
        );

        clawRotation = new PriorityServoV2(
                new Servo[] {robot.hardwareMap.get(Servo.class, "clawRotation")},
                "clawRotation",
                PriorityServoV2.ServoType.SPEED,
                1.0,
                new boolean[] {false},
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
                new boolean[] {false},
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

    public void setHorizontalPos(double targetRad){
        double wheelRot = targetRad/wheelRadius;
        double bigGearsTraversed = wheelRot * smallGearNum * 0.5;
        double bigGearRot = bigGearsTraversed / bigGearNum;

        horizontalRail.setTargetAngle(bigGearRot);
    }

    public double getHorizontalPos(){
        double currRad = horizontalRail.getCurrentAngle();
        double smallGearsTraversed = currRad/(Math.PI * 2) * bigGearNum * 2;
        double wheelRot = smallGearsTraversed/smallGearNum;

        return wheelRot * 2 * Math.PI * wheelRadius;
    }

    public boolean inPosition(){
        //TODO: Figure out what actually is essential
        return horizontalRail.inPosition() && armRotation.inPosition() && clawRotation.inPosition() && clawGrip.inPosition();
    }
}
