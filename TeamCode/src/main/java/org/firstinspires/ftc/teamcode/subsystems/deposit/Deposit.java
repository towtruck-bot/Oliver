package org.firstinspires.ftc.teamcode.subsystems.deposit;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.sensors.Sensors;

public class Deposit {
    public enum State{
        IDLE, //resting position everything in retracted state
        EXTEND_SAMPLE, //extend claw arm while raising linear rail
        EXTEND_SPECIMEN, //extend claw arm while raising linear rail
        START_DEPOSIT_SAMPLE, //rotate claw downwards and release
        FINISH_DEPOSIT_SAMPLE, //rotate claw upwards
        START_DEPOSIT_SPECIMEN, //rotate claw downwards and release
        FINISH_DEPOSIT_SPECIMEN, //rotate claw upwards
        RETRACT, //retract claw back while lowering linear rail
    };
    public State state;

    public Robot robot;
    public Slides slides;
    public Arm arm;
    public Sensors sensors;

    public final double initAngle = Math.toRadians(180.0);

    public final double sampleDepositAngle = Math.toRadians(60.0);
    public final double sampleDropHeight = 1.0;

    public final double specimenHoldAngle = Math.toRadians(150.0);
    public final double specimenAttachHeight = 0.5;
    
    public Deposit(Robot robot){
        this.robot = robot;
        this.sensors = robot.sensors;
        this.slides = robot.slides;

        arm = new Arm(robot);

        state = State.IDLE;
    }
}
