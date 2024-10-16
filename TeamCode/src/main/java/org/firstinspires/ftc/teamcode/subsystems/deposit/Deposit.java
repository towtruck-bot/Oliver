package org.firstinspires.ftc.teamcode.subsystems.deposit;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.sensors.Sensors;

public class Deposit {
    public enum State{
        IDLE,
        TRANSFER,
        HOLD,
        RETURN,
        GRAB,
        SAMPLEREADY,
        SAMPLER,
        SAMPLED,
        SPECIREADY,
        SPECIR,
        SPECID
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

    private final double robotBaseHeight = 6.0;
    private final double armLength = 5.0;

    private boolean outtake = false;
    private boolean intakefinished = false;
    private boolean transferStart = false;

    //start sample -> sample ready(good to start R) -> sampled(start actual deposit)
    private boolean ssample = false;
    private boolean sampleready = false;
    private boolean sampled = false;

    //start speci -> speci ready(good to start R) -> specid(start actual deposit)
    private boolean sspeci = false;
    private boolean speciready = false;
    private boolean specid = false;

    private final double idleArmAngle = Math.toRadians(0.0);
    private final double idleArmPos = 0.0;
    private final double idleClawAngle = Math.toRadians(0.0);
    private final double idleSlidesHeight = 0.0;

    private final double transferArmAngle = Math.toRadians(-60.0);
    private final double transferArmPos = 11.811 - armLength * Math.cos(transferArmAngle);
    private final double transferClawAngle = Math.toRadians(-30.0);
    private final double transferSlidesHeight = 0.0;

    private final double sampleReadyArmAngle = Math.toRadians(90.0);
    private final double sampleReadyArmPos = 0.0;
    private final double sampleReadyClawAngle = Math.toRadians(0.0);
    private final double sampleReadySlidesHeight = 0.0;

    private final double sampleRArmAngle = Math.toRadians(135.0);
    private final double sampleRArmPos = 0.0;
    private final double sampleRClawAngle = Math.toRadians(0.0);
    private final double sampleRSlidesHeight = 46 - armLength * Math.sin(sampleRArmAngle) - robotBaseHeight;

    private final double sampleDArmAngle = Math.toRadians(135.0);
    private final double sampleDArmPos = 0.0;
    private final double sampleDClawAngle = Math.toRadians(45.0);
    private final double sampleDSlidesHeight = 46 - armLength * Math.sin(sampleDArmAngle) - robotBaseHeight;

    private final double speciReadyArmAngle = Math.toRadians(90.0);
    private final double speciReadyArmPos = 11.811;
    private final double speciReadyClawAngle = Math.toRadians(0.0);
    private final double speciReadySlidesHeight = 0.0;

    private final double speciRArmAngle = Math.toRadians(90.0);
    private final double speciRArmPos = 11.811;
    private final double speciRClawAngle = Math.toRadians(-30.0);
    private final double speciRSlidesHeight = 26.0 - armLength - 2.0;

    private final double speciDArmAngle = Math.toRadians(0.0);
    private final double speciDArmPos = 11.811;
    private final double speciDClawAngle = Math.toRadians(-30.0);
    private final double speciDSlidesHeight = 26.0 - armLength;

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
                arm.setArmAngle(idleArmAngle);
                arm.setMgnPosition(idleArmPos);
                arm.setClawAngle(idleClawAngle);
                slides.setTargetLength(idleSlidesHeight);
                if(transferStart && arm.checkReady()){
                    state = State.TRANSFER;
                    transferStart = false;
                }
                if(outtake){
                    state = State.HOLD;
                    outtake = false;
                }
            case TRANSFER:
                arm.setArmAngle(transferArmAngle);
                arm.setMgnPosition(transferArmPos);
                arm.setClawAngle(transferClawAngle);
                slides.setTargetLength(transferSlidesHeight);

                if(ssample && arm.checkReady()){
                    state = State.SAMPLEREADY;
                    ssample = false;
                }
                if(sspeci && arm.checkReady()){
                    state = State.SPECIREADY;
                    sspeci = false;
                }
            case SAMPLEREADY:
                arm.setArmAngle(sampleReadyArmAngle);
                arm.setMgnPosition(sampleReadyArmPos);
                arm.setClawAngle(sampleReadyClawAngle);
                slides.setTargetLength(sampleReadySlidesHeight);
                if(sampleready && arm.checkReady() && slides.getLength() == sampleReadySlidesHeight){
                    state = State.SAMPLER;
                    sampleready = false;
                }
            case SAMPLER:
                arm.setArmAngle(sampleRArmAngle);
                arm.setMgnPosition(sampleRArmPos);
                arm.setClawAngle(sampleRClawAngle);
                slides.setTargetLength(sampleRSlidesHeight);
                if(sampled && arm.checkReady() && slides.getLength() == sampleRSlidesHeight){
                    state = State.SAMPLED;
                    sampled = false;
                }
            case SAMPLED:
                arm.setArmAngle(sampleDClawAngle);
                arm.setMgnPosition(sampleDClawAngle);
                arm.setClawAngle(sampleDClawAngle);
                slides.setTargetLength(sampleDSlidesHeight);
                if(arm.checkReady()){
                    state = State.IDLE;
                }
            case HOLD:
                arm.setArmAngle(idleArmAngle);
                arm.setMgnPosition(idleArmPos);
                arm.setClawAngle(idleClawAngle);
                slides.setTargetLength(idleSlidesHeight);
                if(intakefinished){
                    state = State.RETURN;
                    intakefinished = false;
                }
            case RETURN:
                //would this be intake technically? considering the  mechanism used is the intake
                //to chuck the sample out
            case GRAB:
                //insert math here to move arm certain position + claw to grab
                //would i need to access the distance sensor here? if so, how would i
                if(arm.checkReady()){
                    state = State.SPECIREADY;
                }
            case SPECIREADY:
                arm.setArmAngle(speciReadyArmAngle);
                arm.setMgnPosition(speciReadyArmPos);
                arm.setClawAngle(speciReadyClawAngle);
                slides.setTargetLength(speciReadySlidesHeight);
                if(speciready && arm.checkReady() && slides.getLength() == speciReadySlidesHeight){
                    state = State.SPECIR;
                    speciready = false;
                }
            case SPECIR:
                arm.setArmAngle(speciRArmAngle);
                arm.setMgnPosition(speciRArmPos);
                arm.setClawAngle(speciRClawAngle);
                slides.setTargetLength(speciRSlidesHeight);
                if(specid && arm.checkReady() && slides.getLength() == speciRSlidesHeight){
                    state = State.SPECID;
                    specid = false;
                }
            case SPECID:
                arm.setArmAngle(speciDArmAngle);
                arm.setMgnPosition(speciDArmPos);
                arm.setClawAngle(speciDClawAngle);
                slides.setTargetLength(speciDSlidesHeight);
                if(arm.checkReady()){
                    state = State.IDLE;
                }
        }
    }

    public void startTransfer(){ transferStart = true;}

    public void startOuttake(){outtake = true;}

    public void setSampleReady(){sampleready = true;}

    public void setSpeciReady(){speciready = true;}

    public void startSample(){ssample = true;}

    public void startSpeci(){sspeci = true;}

    public void startSampleD(){sampled = true;}

    public void startSpeciD(){specid = true;}
}
