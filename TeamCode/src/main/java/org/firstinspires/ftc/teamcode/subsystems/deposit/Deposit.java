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

    public final double initArmAngle = Math.toRadians(180.0);
    public final double initClawAngle = Math.toRadians(180.0);
    public final double initDiffyR = Math.toRadians(180.0);
    public final double initDiffyL = Math.toRadians(180.0);


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
    public double sample_rotation = 0.0; // update these values
    public final double sample_spin = 0.0; // update these values
    public final double specimen_rotation = 0.0; // update these values
    public final double specimen_spin = 0.0; // update these values
    public final double clawOpenAngle = Math.toRadians(0.0);


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
                arm.armRotation.setTargetAngle(initArmAngle,1.0);
                arm.clawActuation.setTargetAngle(initClawAngle, 1.0);
                arm.mgnLinkage.setTargetPose(initMGNPos, 1.0); //i needa do math for this but im not thinking rn - James
                arm.diffyR.setTargetPose(initDiffyR, 1.0);
                arm.diffyL.setTargetAngle(initDiffyL, 1.0);
                slides.setTargetLength(initRaiseHeight);
            case RAISEDSAMPLE:
                arm.armRotation.setTargetAngle(sampleArmAngle,1.0);
                arm.clawActuation.setTargetAngle(sampleClawAngle,1.0);
                arm.mgnLinkage.setTargetPose(sampleMGNPos, 1.0);
                slides.setTargetLength(sampleRaiseHeight);
                if(arm.checkReady()){state = State.DEPOSITSAMPLE;}
            case DEPOSITSAMPLE:
                arm.diffyL.setTargetAngle(sample_rotation+sample_spin, 1.0);
                arm.diffyR.setTargetAngle(sample_rotation-sample_spin, 1.0);
                arm.clawActuation.setTargetAngle(clawOpenAngle, 1.0);
                state = State.IDLE;
            case RAISEDSPECIMEN:
                arm.armRotation.setTargetAngle(specimenHoldAngle, 1.0);
                arm.clawActuation.setTargetAngle(specimenClawAngle, 1.0);
                arm.mgnLinkage.setTargetPose(specimenMGNPos, 1.0);
                slides.setTargetLength(specimenRaiseHeight);
                if(arm.checkReady()){state = State.DEPOSITSPECIMEN;}
            case DEPOSITSPECIMEN:
                slides.setTargetLength(specimenDepositHeight);
                arm.diffyL.setTargetAngle(specimen_rotation+specimen_spin, 1.0);
                arm.diffyR.setTargetAngle(specimen_rotation-specimen_spin, 1.0);
                arm.clawActuation.setTargetAngle(clawOpenAngle, 1.0);
                state = State.IDLE;
        }
    }
}
