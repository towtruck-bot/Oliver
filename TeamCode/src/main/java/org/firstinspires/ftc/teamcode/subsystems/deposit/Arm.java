package org.firstinspires.ftc.teamcode.subsystems.deposit;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.sensors.Sensors;
import org.firstinspires.ftc.teamcode.utils.Utils;
import org.firstinspires.ftc.teamcode.utils.priority.PriorityServo;
import org.firstinspires.ftc.teamcode.utils.priority.PriorityServoV2;
import org.firstinspires.ftc.teamcode.utils.priority.nPriorityServo;

public class Arm {
    public final Sensors sensors;

//    public final PriorityServo horizontalRail;
//    public final PriorityServo armRotation;
//    public final PriorityServo clawRotation;
//    public final PriorityServo clawGrip;

    public final nPriorityServo horizontalRail;
    public final nPriorityServo armRotation;
    public final nPriorityServo clawRotation;
    public final nPriorityServo clawGrip;

    private final double wheelRadius = 1.75;
    private final double smallGearNum = 40.0, bigGearNum = 25.0;
    public final double armLength = 5.905314961;
    public static final double armLengthT = 5.905314961;

    private double samplePrepareRad = 0.0, sampleGrabRad = 0.2544, speciPrepareRad = 0.7123, speciGrabRad = 0;

    public Arm(Robot robot){
        horizontalRail = new nPriorityServo(
                new Servo[] {robot.hardwareMap.get(Servo.class, "horizontalRail")},
                "horizontalRail",
                nPriorityServo.ServoType.AXON_MINI,
                0.0,
                0.535,
                0.533,
                new boolean[] {false},
                1.0,
                2.0
        );

        robot.hardwareQueue.addDevice(horizontalRail);

        armRotation = new nPriorityServo(
                new Servo[] {robot.hardwareMap.get(Servo.class, "armRotationL"), robot.hardwareMap.get(Servo.class, "armRotationR")},
                "armRotation",
                nPriorityServo.ServoType.AXON_MINI,
                0.0,
                1.0,
                0.0,
                new boolean[] {true, false},
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
                0.272,
                new boolean[] {false},
                1.0,
                2.0
        );

        robot.hardwareQueue.addDevice(clawRotation);

        clawGrip = new nPriorityServo(
                new Servo[] {robot.hardwareMap.get(Servo.class, "clawGrip")},
                "clawGrip",
                nPriorityServo.ServoType.AXON_MINI,
                0.0,
                0.219,
                0.0,
                new boolean[] {false},
                1.0,
                2.0
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
        double currRad = -1 * horizontalRail.getCurrentAngle();
        double smallGearsTraversed = currRad/(Math.PI * 2) * bigGearNum * 2;
        double wheelRot = smallGearsTraversed/smallGearNum;

        return wheelRot * 2 * Math.PI * wheelRadius;
    }

    public void setArmRotation(double rad, double pow){
        armRotation.setTargetAngle(rad, pow);
    }

    public double getArmRotation(){
        return armRotation.getCurrentAngle();
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