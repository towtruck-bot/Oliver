package org.firstinspires.ftc.teamcode.subsystems.deposit;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.sensors.Sensors;
import org.firstinspires.ftc.teamcode.utils.Utils;
import org.firstinspires.ftc.teamcode.utils.priority.PriorityServo;

public class Arm {
    public final Sensors sensors;
    public final PriorityServo mgnLinkage;
    public final PriorityServo armRotation;
    public final PriorityServo diffyL, diffyR;
    public final PriorityServo clawActuation;

    private final double mgnArmDriving = 6.29921, mgnArmDriven = 4.48819;
    private final double horiShift = 1.0, vertShift = 1.0; //TODO: Update these values
    private final double armLength = 1.0; //TODO: Update these values

    public Arm(Robot robot){
        mgnLinkage = new PriorityServo(
                new Servo[] {hardwareMap.get(Servo.class, "mgnServoL"), hardwareMap.get(Servo.class,"mgnServoR")},
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
        robot.hardwareQueue.addDevice(mgnLinkage);

        armRotation = new PriorityServo(
                new Servo[] {hardwareMap.get(Servo.class, "armServoL"), hardwareMap.get(Servo.class, "armServoR")},
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
        robot.hardwareQueue.addDevice(armRotation);

        diffyL = new PriorityServo(
                new Servo[] {hardwareMap.get(Servo.class, "diffyL")},
                "diffyL",
                PriorityServo.ServoType.SPEED,
                1.0,
                0.0,
                1.0,
                0.0,
                true,
                3.0,
                5.0,
                new double[] {1.0}
        );
        robot.hardwareQueue.addDevice(diffyL);

        diffyR = new PriorityServo(
                new Servo[] {hardwareMap.get(Servo.class, "diffyR")},
                "diffyR",
                PriorityServo.ServoType.SPEED,
                1.0,
                0.0,
                1.0,
                0.0,
                false,
                3.0,
                5.0,
                new double[] {1.0}
        );
        robot.hardwareQueue.addDevice(diffyR);

        clawActuation = new PriorityServo(
                new Servo[] {hardwareMap.get(Servo.class, "claw")},
                "clawActuation",
                PriorityServo.ServoType.SPEED,
                1.0,
                0.0,
                1.0,
                0.0,
                false,
                3.0,
                5.0,
                new double[] {1.0}
        );
        robot.hardwareQueue.addDevice(clawActuation);

        this.sensors = robot.sensors;
    }

    public void update(){
        diffyL.setTargetAngle(diffyL.getCurrentAngle() - armRotation.getCurrentAngle() / 19.0, 1.0);
        diffyR.setTargetAngle(diffyR.getCurrentAngle() - armRotation.getCurrentAngle() / 19.0, 1.0);
    }

    public void setArmAngle(double rad){
        armRotation.setTargetAngle(rad, 1.0);
    }

    //rotation: up down
    //spin: left right
    public void setDiffy(double rotation, double spin){
        diffyL.setTargetAngle(rotation+spin, 1.0);
        diffyR.setTargetAngle(rotation-spin, 1.0);
    }

    public void openClaw(){
        clawActuation.setTargetAngle(Math.toRadians(135.0), 1.0);
    }

    //TODO: Update this value
    public void closeClaw(){
        clawActuation.setTargetAngle(Math.toRadians(15.0), 1.0);
    }

    public void setMgnPosition(double newPos){
        newPos = Utils.minMaxClip(newPos, 0.0, 11.811);
        double c = Math.sqrt(Math.pow(newPos + horiShift, 2) + Math.pow(vertShift, 2));
        double alpha = Math.atan2(newPos + horiShift, vertShift);
        double targetAngle = Math.acos((Math.pow(mgnArmDriving, 2) + Math.pow(c, 2) - Math.pow(mgnArmDriven, 2))/(2 * mgnArmDriving * c)) - alpha;
        mgnLinkage.setTargetAngle(targetAngle, 1.0);
    }

    public double calcMgnPosition(){
        double drivingArmX = mgnArmDriving * Math.cos(mgnLinkage.getCurrentAngle());
        double drivenArmX = Math.sqrt(Math.pow(mgnArmDriven, 2) - Math.pow((vertShift + mgnArmDriving*Math.sin(mgnLinkage.getCurrentAngle())), 2));
        return drivingArmX + drivenArmX - horiShift;
    }

    public double getArmLength(){
        return armLength;
    }

    public double getArmAngle(){
        return clawActuation.getCurrentAngle();
    }

    public boolean checkReady(){
        return mgnLinkage.inPosition() && armRotation.inPosition() && diffyR.inPosition() && clawActuation.inPosition();
    }
}
