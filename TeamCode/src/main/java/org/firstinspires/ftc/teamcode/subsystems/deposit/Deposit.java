package org.firstinspires.ftc.teamcode.subsystems.deposit;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.sensors.Sensors;
import org.firstinspires.ftc.teamcode.utils.priority.HardwareQueue;

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

    public static double armInitAngle = 0.0;
    public static double armGrabAngle = 0.0;
    public static double armDepositAngle = 0.0;

    public static double clawInitAngle = 0.0;
    public static double clawGrabAngle = 0.0;
    public static double clawDepositAngle = 0.0;
    
    public Deposit(Robot robot){
        this.robot = robot;
        this.sensors = robot.sensors;
        this.slides = robot.slides;

        arm = new Arm(robot);

        state = State.IDLE;
    }
}
