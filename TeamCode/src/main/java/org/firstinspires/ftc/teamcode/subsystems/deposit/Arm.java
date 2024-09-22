package org.firstinspires.ftc.teamcode.subsystems.deposit;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.sensors.Sensors;
import org.firstinspires.ftc.teamcode.utils.priority.PriorityServo;

public class Arm {
    public final Sensors sensors;
    public final PriorityServo baseMovement;
    public final PriorityServo armAngle;

    public Arm(Robot robot){
        //Talked with Neil
        //There are two servos moving the whole v4 bar along the rail. Defined as baseMovement
        //There are two servos controlling the angle of the arm. Defined as armAngle
        //Should be one more servo controlling claw angle itself for more detailed adjustments, need to ask Ryan if he is using that
        // - James
        Servo[] base = new Servo[] {hardwareMap.get(Servo.class, "V4BarServo1"), hardwareMap.get(Servo.class, "V4BarServo2")};
        baseMovement = new PriorityServo(
                base,
                "baseMovement",
                PriorityServo.ServoType.SPEED,
                0.929999,
                0.0,
                1.0,
                0.0,
                false,
                3.0,
                5.0,
                new double[] {-1.0, 1.0}
        );

        Servo[] angle = new Servo[] {hardwareMap.get(Servo.class, "V4BarServo3"), hardwareMap.get(Servo.class, "V4BarServo4")};
        armAngle = new PriorityServo(
                angle,
                "armAngle",
                PriorityServo.ServoType.SPEED,
                0.92999,
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
        return baseMovement.inPosition() && armAngle.inPosition();
    }
}
