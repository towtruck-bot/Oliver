package org.firstinspires.ftc.teamcode.subsystems.deposit;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.sensors.Sensors;

public class Deposit {
    public enum State{
        IDLE, //resting position everything in retracted state
        RAISEDSAMPLE,
        DEPOSITSAMPLE,
        RAISEDSPECIMEN,
        DEPOSITSPECIMEN
    };
    public State state;

    public Robot robot;
    public Slides slides;
    public Arm arm;
    public Sensors sensors;

    // Arm Servos
    // Referring to the default position in this doc: https://docs.google.com/document/d/1J4mOFkPJLNY-hA3XU2I-XiUrz45ra0IlPodfkXszulE/edit?usp=sharing
    // Let the intake side be defined as the back and deposit as the front
    // The Mgn base + rail will be the x - axis
    // 0 degrees will be in the positive direction(i.e. arm pointed straight out of the deposit along mgn rail is 0deg)
    // mgn pos will begin at 0.0 length, currently defined as the base in between the vertical slides
    public final double initArmAngle = Math.toRadians(180.0);
    public final double initClawAngle = Math.toRadians(180.0);
    public final double initRaiseHeight = 0.0;
    public final double initMGNPos = 0.0;

    public final double sampleArmAngle = Math.toRadians(60.0);
    public final double sampleClawAngle = Math.toRadians(60.0);
    public final double sampleRaiseHeight = 1.0;
    public final double sampleMGNPos = 0.0;

    public final double specimenHoldAngle = Math.toRadians(150.0);
    public final double specimenClawAngle = Math.toRadians(0.0);
    public final double specimenRaiseHeight = 0.5;
    public final double specimenMGNPos = 0.0;
    public final double specimenDepositHeight = 1.0;
    public final double clawOpenAngle = Math.toRadians(0.0);

    // Diffy Servos
    // Ryan please add your definition of 0 radians here
    public final double initDiffyR = Math.toRadians(180.0);
    public final double initDiffyL = Math.toRadians(180.0);
    public double sample_rotation = 0.0; // update these values
    public final double sample_spin = 0.0; // update these values
    public final double specimen_rotation = 0.0; // update these values
    public final double specimen_spin = 0.0; // update these values


    public Deposit(Robot robot){
        this.robot = robot;
        this.sensors = robot.sensors;
        this.slides = robot.slides;

        arm = new Arm(robot);

        state = State.IDLE;
    }

    public void update(){
        switch(state){
            case IDLE:
                arm.setArmAngle(initArmAngle);
                arm.setClawAngle(initClawAngle);
                arm.setMgnPosition(initMGNPos);
                arm.setDiffy(initDiffyR, initDiffyL);
                slides.setTargetLength(initRaiseHeight);

            case RAISEDSAMPLE:
                arm.setArmAngle(sampleArmAngle);
                arm.setClawAngle(sampleClawAngle);
                arm.setMgnPosition(sampleMGNPos);
                slides.setTargetLength(sampleRaiseHeight);
                if(arm.checkReady()){state = State.DEPOSITSAMPLE;}

            case DEPOSITSAMPLE:
                arm.setDiffy(sample_rotation - sample_spin, sample_rotation + sample_spin);
                arm.setClawAngle(clawOpenAngle);
                state = state.IDLE;

            case RAISEDSPECIMEN:
                arm.setArmAngle(specimenHoldAngle);
                arm.setClawAngle(specimenClawAngle);
                arm.setMgnPosition(specimenMGNPos);
                slides.setTargetLength(specimenRaiseHeight);
                if(arm.checkReady()){state = State.DEPOSITSPECIMEN;}

            case DEPOSITSPECIMEN:
                slides.setTargetLength(specimenDepositHeight);
                arm.setDiffy(specimen_rotation - specimen_spin, specimen_rotation + specimen_rotation);
                arm.setClawAngle(clawOpenAngle);
                state = State.IDLE;
        }
    }
}
