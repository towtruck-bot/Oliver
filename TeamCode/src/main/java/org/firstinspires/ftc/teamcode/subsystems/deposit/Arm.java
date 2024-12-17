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

    public final PriorityServo horizontalRail;
    public final PriorityServo armRotation;
    public final PriorityServo clawRotation;
    public final PriorityServo clawGrip;

//    public final PriorityServoV2 horizontalRail;
//    public final PriorityServoV2 armRotation;
//    public final PriorityServoV2 clawRotation;
//    public final PriorityServoV2 clawGrip;

    private final double wheelRadius = 1.75;
    private final double smallGearNum = 25.0, bigGearNum = 40.0;
    public final double armLength = 5.905314961;

    private double currArmRotation = 0.0;
    //TODO: Get values below, current values are guesses
    private double samplePrepareRad = 0.0, sampleGrabRad = Math.PI, speciPrepareRad = Math.PI, speciGrabRad = Math.PI/6;

    public Arm(Robot robot){
//        horizontalRail = new PriorityServoV2(
//                new Servo[] {robot.hardwareMap.get(Servo.class, "horizontalRail")},
//                "horizontalRail",
//                PriorityServoV2.ServoType.SPEED,
//                1.0,
//                new boolean[] {false},
//                0.0,
//                1.0,
//                0.0,
//                0.0,
//                1.0,
//                3.0,
//                5.0
//        );

        horizontalRail = new PriorityServo(
                new Servo[] {robot.hardwareMap.get(Servo.class, "horizontalRail")},
                "horizontalRail",
                PriorityServo.ServoType.SPEED,
                1.0,
                0.0,
                0.542,
                0.0,
                true,
                1.0,
                2.0,
                new double[] {1.0}
        );

        robot.hardwareQueue.addDevice(horizontalRail);

//        armRotation = new PriorityServoV2(
//                new Servo[] {robot.hardwareMap.get(Servo.class, "armRotationL"), robot.hardwareMap.get(Servo.class, "armRotationR")},
//                "armRotation",
//                PriorityServoV2.ServoType.SPEED,
//                1.0,
//                new boolean[] {false, true}, //TODO: Check these - James
//                0.0,
//                1.0,
//                0.0,
//                0.0,
//                1.0,
//                3.0,
//                5.0
//        );

        armRotation = new PriorityServo(
                new Servo[] {robot.hardwareMap.get(Servo.class, "armRotationL"), robot.hardwareMap.get(Servo.class, "armRotationR")},
                "armRotation",
                PriorityServo.ServoType.SPEED,
                1.0,
                0.0,
                1.0,
                0.0,
                false,
                1.0,
                2.0,
                new double[] {1.0, -1.0}
        );

        robot.hardwareQueue.addDevice(armRotation);

//        clawRotation = new PriorityServoV2(
//                new Servo[] {robot.hardwareMap.get(Servo.class, "clawRotation")},
//                "clawRotation",
//                PriorityServoV2.ServoType.SPEED,
//                1.0,
//                new boolean[] {false},
//                0.0,
//                1.0,
//                0.0,
//                0.0,
//                1.0,
//                3.0,
//                5.0
//        );

        clawRotation = new PriorityServo(
                new Servo[] {robot.hardwareMap.get(Servo.class, "clawRotation")},
                "clawRotation",
                PriorityServo.ServoType.SPEED,
                1.0,
                0.0,
                0.6,
                0.0,
                false,
                1.0,
                2.0,
                new double[]{1.0}
        );

        robot.hardwareQueue.addDevice(clawRotation);

//        clawGrip = new PriorityServoV2(
//                new Servo[] {robot.hardwareMap.get(Servo.class, "clawGrip")},
//                "clawGrip",
//                PriorityServoV2.ServoType.SPEED,
//                1.0,
//                new boolean[] {false},
//                0.0,
//                1.0,
//                0.0,
//                0.0,
//                1.0,
//                3.0,
//                5.0
//        );

        clawGrip = new PriorityServo(
                new Servo[] {robot.hardwareMap.get(Servo.class, "clawGrip")},
                "clawGrip",
                PriorityServo.ServoType.SPEED,
                1.0,
                0.0,
                0.21,
                0.0,
                false,
                1.0,
                2.0,
                new double[] {1.0}
        );

        robot.hardwareQueue.addDevice(clawGrip);

        this.sensors = robot.sensors;
    }

    public void setHorizontalPos(double targetRad, double pow){
        double wheelRot = targetRad/wheelRadius;
        double bigGearsTraversed = wheelRot * smallGearNum * 0.5;
        double bigGearRot = bigGearsTraversed / bigGearNum;

        horizontalRail.setTargetAngle(bigGearRot, pow);
    }

    public double getHorizontalPos(){
        double currRad = horizontalRail.getCurrentAngle();
        double smallGearsTraversed = currRad/(Math.PI * 2) * bigGearNum * 2;
        double wheelRot = smallGearsTraversed/smallGearNum;

        return wheelRot * 2 * Math.PI * wheelRadius;
    }

    public void setArmRotation(double rad, double pow){
        armRotation.setTargetAngle(rad, pow);
        currArmRotation = rad;
    }

    public double getArmRotation(){
        return currArmRotation;
    }

    public void setClawRotation(double rad, double pow){
        clawRotation.setTargetAngle(rad, pow);
    }

    public void setClawSamplePrepare(){
        clawGrip.setTargetAngle(samplePrepareRad, 1.0);
    }

    public void setClawSampleGrab(){
        clawGrip.setTargetAngle(sampleGrabRad, 1.0);
    }

    public void setClawSpeciPrepare(){
        clawGrip.setTargetAngle(speciPrepareRad, 1.0);
    }

    public void setClawSpeciGrab(){
        clawGrip.setTargetAngle(speciGrabRad, 1.0);
    }

    public boolean inPosition(){
        //TODO: Figure out what actually is essential
        return horizontalRail.inPosition() && armRotation.inPosition() && clawRotation.inPosition() && clawGrip.inPosition();
    }
}