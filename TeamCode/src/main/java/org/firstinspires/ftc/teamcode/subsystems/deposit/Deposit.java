package org.firstinspires.ftc.teamcode.subsystems.deposit;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.sensors.Sensors;
import org.firstinspires.ftc.teamcode.utils.Utils;

public class Deposit {
    public enum State{
        IDLE,
        TRANSFER_START,
        TRANSFER_END,
        SAMPLE_READY,
        SAMPLE_RAISE,
        SAMPLE_DEPOSIT,
        GRAB,
        SPECIMEN_READY,
        SPECIMEN_RAISE,
        SPECIMEN_DEPOSIT,
        BUFFER,
        RETRACT
    };
    public State state;

    public Robot robot;
    public Slides slides;
    public Arm arm;
    public Sensors sensors;

    private double currX, currY, currArmAngle, currClawAngle;
    private double targetX, targetY, targetClawAngle;
    private double moveToX, moveToY, moveToArmAngle;
    private boolean tooClose = false;

    private final double intakeX = 10.0, intakeY = -2.0, intakeAngleMin = Math.toRadians(-90.0), intakeAngleMax = Math.toRadians(0); // TODO: Update these values
    private final double sampleBasketX = -2.0, sampleBasketY = 46.0;
    private final double specimenBarX = 10.0, specimenBarY = 27.0;

    public Deposit(Robot robot){
        this.robot = robot;
        this.sensors = robot.sensors;
        this.slides = new Slides(robot);

        arm = new Arm(robot);

        this.currX = 0.0;
        this.currY = arm.getArmLength();
        this.currArmAngle = Math.toRadians(90.0); //TODO: Determine correct starting values, also if claw should be relative to arm or overall angle(imo overall angle but discuss)
        this.currClawAngle = Math.toRadians(0.0);

        state = State.IDLE;
    }

    public void update(){
        switch(state){
            case IDLE:
                break;
            case TRANSFER_START:
                setDepositPositions(intakeX, intakeY, Math.toRadians(0.0));

                calculateMoveToWithRestrictions(intakeAngleMin, intakeAngleMax);

                if((targetX - currX) * (targetX - currX) + (targetY - currY) * (targetY - currY) <= arm.getArmLength() * arm.getArmLength()){
                    tooClose = true;
                    arm.setMgnPosition(moveToX);
                    slides.setTargetLength(moveToY);
                }else{
                    arm.setArmAngle(moveToArmAngle);
                    arm.setClawAngle(targetClawAngle);
                }

                if(arm.checkReady()){
                    state = State.TRANSFER_END;
                }
                break;
            case TRANSFER_END:
                if(tooClose){
                    arm.setArmAngle(moveToArmAngle);
                    arm.setClawAngle(targetClawAngle);
                }else{
                    arm.setMgnPosition(moveToX);
                    slides.setTargetLength(moveToY);
                }

                if(arm.checkReady() && slides.inPosition(0.5)){
                    state = State.SAMPLE_READY;
                    currX = targetX;
                    currY = targetY;
                    currArmAngle = moveToArmAngle;
                    currClawAngle = targetClawAngle;
                }
                break;
            case SAMPLE_READY:
                //confirm grabbed, use color sensor?
                //if grab confirmed, move to next state
                break;
            case SAMPLE_RAISE:
                setDepositPositions(sampleBasketX, sampleBasketY + 2.0, Math.toRadians(225.0));
                calculateMoveTo();

                arm.setMgnPosition(moveToX);
                slides.setTargetLength(moveToY);
                arm.setArmAngle(moveToArmAngle);
                arm.setClawAngle(targetClawAngle - moveToArmAngle);

                if(arm.checkReady() && slides.inPosition(0.5)){
                    state = State.SAMPLE_DEPOSIT;
                    currX = targetX;
                    currY = targetY;
                    currArmAngle = moveToArmAngle;
                    currClawAngle = targetClawAngle;
                }
                break;
            case SAMPLE_DEPOSIT:
                //claw code here
                //goes to buffer
                break;
            case GRAB:
                //claw code here
                //use color sensor to determine where clipped is
                //use claw to grab
                //should this part drive the robot here too?
                break;
            case SPECIMEN_READY:
                //check if grabbed
                //if not go back to grab
                //else go to raise
                break;
            case SPECIMEN_RAISE:
                setDepositPositions(specimenBarX, specimenBarY - 2.0, Math.toRadians(90.0));
                calculateMoveTo();

                arm.setMgnPosition(moveToX);
                slides.setTargetLength(moveToY);
                arm.setArmAngle(moveToArmAngle);
                arm.setClawAngle(targetClawAngle - moveToArmAngle);

                if(arm.checkReady() && slides.inPosition(0.5)){
                    state = State.SPECIMEN_DEPOSIT;
                    currX = targetX;
                    currY = targetY;
                    currArmAngle = moveToArmAngle;
                    currClawAngle = targetClawAngle;
                }
                break;
            case SPECIMEN_DEPOSIT:
                setDepositPositions(specimenBarX, specimenBarY, Math.toRadians(90.0));
                calculateMoveTo();

                arm.setMgnPosition(moveToX);
                slides.setTargetLength(moveToY);
                arm.setArmAngle(moveToArmAngle);
                arm.setClawAngle(targetClawAngle - moveToArmAngle);

                if(arm.checkReady() && slides.inPosition(0.5)){
                    state = State.BUFFER;
                    currX = targetX;
                    currY = targetY;
                    currArmAngle = moveToArmAngle;
                    currClawAngle = targetClawAngle;
                }
                break;
            case BUFFER:
                //facing back
                if(currArmAngle > Math.toRadians(90.0)){
                    setDepositPositions(currX + 1.0, currY - 1.0);
                }else{
                    //open claw too btw
                    setDepositPositions(currX - 1.0, currY - 1.0);
                }
                calculateMoveTo();

                arm.setMgnPosition(moveToX);
                slides.setTargetLength(moveToY);
                arm.setArmAngle(moveToArmAngle);
                arm.setClawAngle(targetClawAngle - moveToArmAngle);

                if(arm.checkReady() && slides.inPosition(0.5)){
                    state = State.RETRACT;
                    currX = targetX;
                    currY = targetY;
                    currArmAngle = moveToArmAngle;
                    currClawAngle = targetClawAngle;
                }
                break;
            case RETRACT:
                setDepositPositions(0.0, arm.getArmLength(), 0.0);
                calculateMoveTo();

                arm.setMgnPosition(moveToX);
                slides.setTargetLength(moveToY);
                arm.setArmAngle(moveToArmAngle);
                arm.setClawAngle(targetClawAngle - moveToArmAngle);

                if(arm.checkReady() && slides.inPosition(0.5)){
                    state = State.IDLE;
                    currX = targetX;
                    currY = targetY;
                    currArmAngle = moveToArmAngle;
                    currClawAngle = targetClawAngle;
                }
                break;
        }
    }

    public void startTransfer() {
        // TODO Fill in method stub
        state = State.TRANSFER_START;
    }

    public boolean isSampleReady() {
        // TODO Fill in method stub
        return true;
    }

    public void startSampleDeposit() {
        // TODO Fill in method stub
    }

    public boolean isSampleDepositDone() {
        // TODO Fill in method stub
        return true;
    }

    public void startOuttake() {
        // TODO Fill in method stub
    }

    public boolean isOuttakeDone() {
        // TODO Fill in method stub
        return true;
    }

    public void grabSpecimen() {
        // TODO Fill in method stub
        state = State.GRAB;
    }

    public boolean isSpecimenReady() {
        // TODO Fill in method stub
        return true;
    }

    public void startSpecimenDeposit() {
        // TODO Fill in method stub
    }

    public boolean isSpecimenDepositDone() {
        // TODO Fill in method stub
        return true;
    }

    public void setDepositPositions(double x, double y, double clawAngle){
        targetX = x;
        targetY = y;
        targetClawAngle = clawAngle;
    }

    public void setDepositPositions(double x, double y){
        targetX = x;
        targetY = y;
    }

    //TODO: Update minMaxClip values
    public void calculateMoveTo(){
        double baseX = Utils.minMaxClip(currX + arm.getArmLength() * Math.cos(currArmAngle), 0.0, 11.816);
        double baseY = Utils.minMaxClip(currY + arm.getArmLength() * Math.sin(currArmAngle), 0.0, 50.0);
        double slope = (targetY - baseY)/(targetX - baseX);

        moveToX = Math.sqrt(arm.getArmLength() * arm.getArmLength()/(slope * slope + 1)) + targetX;
        moveToY = slope * (moveToX - targetX) + targetY;
        moveToArmAngle = Math.atan2((targetY - baseY), (targetX - baseX));
    }

    public void calculateMoveToWithRestrictions(double minAngle, double maxAngle){
        double baseX = Utils.minMaxClip(currX + arm.getArmLength() * Math.cos(currArmAngle), 0.0, 11.816);
        double baseY = Utils.minMaxClip(currY + arm.getArmLength() * Math.sin(currArmAngle), 0.0, 50.0);
        double slope = Utils.minMaxClip((targetY - baseY)/(targetX - baseX), minAngle, maxAngle);

        moveToX = Math.sqrt(arm.getArmLength() * arm.getArmLength()/(slope * slope + 1)) + targetX;
        moveToY = slope * (moveToX - targetX) + targetY;
        moveToArmAngle = Math.atan(slope);
    }
}
