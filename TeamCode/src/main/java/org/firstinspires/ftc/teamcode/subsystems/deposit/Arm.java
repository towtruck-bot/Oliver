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
    public final PriorityServo diffyL, diffyR;
    public final PriorityServo clawActuation;

    //assuming both parts of the linkage are the same length, will change if mechanical gives info
    private final double mgnArmLength = 5.90551;
    private final double mgnArmDriving = 6.29921, mgnArmDriven = 4.48819;
    private final double horiShift = 1.0, vertShift = 1.0;

    //private final double mgnArmLengthServo, mgnArmLengthCarriage;
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
                arm,
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

        diffyL = new PriorityServo(
                new Servo[] {hardwareMap.get(Servo.class, "diffyL")},
                "diffyL",
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
                new double[] {-1.0, 1.0}
        );

        Servo[] claw = new Servo[] {hardwareMap.get(Servo.class, "claw")};
        clawActuation = new PriorityServo(
                claw,
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
        return mgnLinkage.inPosition() && armRotation.inPosition() && clawActuation.inPosition();
    }

    public void setArmAngle(double deg){
        armRotation.setTargetAngle(Math.toRadians(deg), 1.0);
    }

    public void setClawAngle(double deg){
        clawActuation.setTargetAngle(Math.toRadians(deg), 1.0);
    }


    // 0 angle is when servo is facing towards intake
    // 0 inches when close to deposit, about 10 inches when close to intake
    public void setMgnPosition(double newPos){
        if(0 < newPos && newPos < 300){
            double d = Math.sqrt((newPos + horiShift) * (newPos + horiShift) + vertShift * vertShift);
            double targetAngle = Math.acos(d/(mgnArmDriven + mgnArmDriving)) + Math.atan2(vertShift, newPos + horiShift);
            mgnLinkage.setTargetAngle(targetAngle, 1.0);
        }
    }

    public double calcMgnPosition(){
        double angle = mgnLinkage.getCurrentAngle();
        double a = 1.0/(mgnArmDriven + mgnArmDriving);
        double b = (2 * horiShift/(mgnArmDriven + mgnArmDriving) - Math.cos(angle));
        double c = (horiShift * horiShift + vertShift * vertShift)/(mgnArmDriven + mgnArmDriving) + Math.sin(angle) * vertShift - Math.cos(angle) * horiShift;
        double ans = (-1.0 * b + Math.sqrt(b * b - 4.0 * a * c))/(2.0 * a);
        return ans;
    }

    public void setDiffy(double degR, double degL){
        diffyR.setTargetAngle(degR, 1.0);
        diffyL.setTargetAngle(degL, 1.0);
    }
}
