package org.firstinspires.ftc.teamcode.subsystems.deposit;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.utils.Globals;

public class Deposit {
    public enum State{
        IDLE,
        TRANSFER_WAIT_ARM,
        TRANSFER_WAIT_CLAW,
        TRANSFER_WAIT,
        TRANSFER_GRAB,
        TRANSFER_CLOSE,
        TRANSFER_FINISH,
        HOLD,
        SAMPLE_RAISE,
        SAMPLE_CLAW,
        SAMPLE_WAIT,
        SAMPLE_DEPOSIT,
        OUTTAKE_MOVE,
        OUTTAKE_RELEASE,
        GRAB_MOVE,
        GRAB_WAIT,
        GRAB,
        GRAB_RETRACT,
        SPECI_RAISE,
        SPECI_WAIT,
        SPECI_DEPOSIT,
        RELEASE,
        RETRACT
    };

    public State state;

    private final Robot robot;
    private final Slides slides;
    private final Arm arm;

    // x values are measured from the base of the arm
    // y values are measured from the ground
    // intake is positive x direction

    // TODO: Re-tune servos
    // arm 0 position is transfer grab position(i.e. at position 0 is the exact location to grab the sample)
    // claw 0 position is perpendicular downwards to that of the arm(i.e. when arm is perfectly horizontal, claw will be facing the ground)

    private final double baseHeight = 10.75, offsetRadArm = Math.PI / 12, offsetRadClaw = Math.PI / 2; //TODO: Get more accurate measurement

    private final double intakeWaitX = 5.905314961 * Math.cos(Math.PI / 24), intakeWaitY = baseHeight + 5.905314961 * Math.sin(Math.PI / 24), intakeX = 5.905314961 * Math.cos(Math.PI / 12), intakeY = baseHeight + 5.905314961 * Math.sin(Math.PI / 12); //TODO: Get more accurate intake locations
    private final double holdX = 5.905314961, holdY = 0.0;
    private final double sampleX = -1.3, sampleLY = 26.0, sampleHY = 44.6;
    private final double outtakeX = 5.905314961, outtakeY = baseHeight;
    private final double speciX = -5.9, speciLSY = 13.5, speciLEY = 15.4, speciHSY = 27.6, speciHEY = 29.6;

    public Deposit(Robot robot){
        this.robot = robot;
        this.slides = new Slides(this.robot);

        arm = new Arm(robot);

        state = Globals.hasSpecimenPreload ? State.HOLD : State.RETRACT;
    }

    public void update(){
        switch(state){
            case IDLE:
                break;
            case TRANSFER_WAIT_ARM:
                moveTo(intakeWaitX, intakeWaitY);
                arm.sampleOpen();

                if(arm.inPosition() && slides.inPosition(0.5)){
                    state = State.TRANSFER_WAIT_CLAW;
                }
                break;
            case TRANSFER_WAIT_CLAW:
                //trying to point the claw straight down(this value will need to be re-tuned), but offsetRadClaw is pointing straight down, and then offsetRadArm is adjusting
                //for the arm's mechanical lower limit
                arm.setClawRotation(offsetRadClaw - offsetRadArm, 1.0); //TODO: Fine tune this

                if(arm.inPosition() && arm.clawFinished()){
                    state = State.TRANSFER_WAIT;
                }
                break;
            case TRANSFER_WAIT:
                break;
            case TRANSFER_GRAB:
                moveTo(intakeX, intakeY);

                if(arm.inPosition() && slides.inPosition(0.5)){
                    state = State.TRANSFER_CLOSE;
                }
                break;
            case TRANSFER_CLOSE:
                arm.sampleClose();

                if(arm.clawFinished()){
                    state = State.TRANSFER_FINISH;
                    robot.clawIntake.release();
                }
                break;
            case TRANSFER_FINISH:
                moveTo(holdX, holdY);

                if(arm.inPosition()){
                    state = State.HOLD;
                }
                break;
            case HOLD:
                break;
            case SAMPLE_RAISE:
                //extending vertical slides
                moveTo(sampleX, sampleHY);

                if(arm.inPosition() && slides.inPosition(0.5)){
                    state = State.SAMPLE_CLAW;
                }
                break;
            case SAMPLE_CLAW:
                arm.setClawRotation(Math.PI - (arm.armRotation.getCurrentAngle() - offsetRadArm) + offsetRadClaw, 1.0);

                if(arm.inPosition()){
                    state = State.SAMPLE_WAIT;
                }
                break;
            case SAMPLE_WAIT:
                break;
            case SAMPLE_DEPOSIT:
                arm.sampleOpen();

                if(arm.clawFinished()){
                    state = State.RETRACT;
                }
                break;
            case OUTTAKE_MOVE:
                moveTo(outtakeX, outtakeY);

                if(arm.inPosition() && slides.inPosition(0.5)){
                    state = State.OUTTAKE_RELEASE;
                }
                break;
            case OUTTAKE_RELEASE:
                arm.sampleOpen();

                if(arm.clawFinished()){
                    state = State.HOLD;
                }
                break;
            case GRAB_MOVE:
                moveTo(outtakeX, outtakeY);
                arm.setClawRotation(arm.armRotation.getCurrentAngle() - offsetRadArm + offsetRadClaw, 1.0);
                arm.speciOpen();

                if(arm.inPosition() && slides.inPosition(0.5) && arm.clawFinished()){
                    state = State.GRAB;
                }
                break;
            case GRAB_WAIT:
                break;
            case GRAB:
                arm.speciClose();

                if(arm.clawFinished()){
                    state = State.GRAB_RETRACT;
                }
                break;
            case GRAB_RETRACT:
                moveTo(holdX, holdY);

                if(arm.inPosition()){
                    state = State.HOLD;
                }
                break;
            case SPECI_RAISE:
                moveTo(speciX, speciHSY);
                arm.setClawRotation(Math.PI - (arm.armRotation.getCurrentAngle() - offsetRadArm) + offsetRadClaw, 1.0);

                if(arm.inPosition() && slides.inPosition(0.5)){
                    state = State.SPECI_WAIT;
                }
                break;
            case SPECI_WAIT:
                break;
            case SPECI_DEPOSIT:
                moveTo(speciX, speciHEY);

                if(arm.inPosition() && slides.inPosition(0.5)){
                    state = State.RELEASE;
                }
                break;
            case RELEASE:
                arm.speciOpen();

                if(arm.clawFinished()){
                    state = State.RETRACT;
                }
                break;
            case RETRACT:
                moveToStart();

                if(arm.inPosition() && slides.inPosition(0.5)){
                    state = State.IDLE;
                }
                break;
        }

        slides.update();
    }

    public void moveTo(double targetX, double targetY){
        double armTargetRad = Math.acos(targetX/arm.armLength) * (targetY >= baseHeight ? 1 : -1);
        double slidesTargetLength = Math.max(targetY - (baseHeight + arm.armLength * Math.sin(armTargetRad)), 0.0);

        arm.setArmRotation(armTargetRad + offsetRadArm, 1.0);
        slides.setTargetLength(slidesTargetLength);
    }

    public double getCurrentX(){
        return Math.cos(arm.armRotation.getCurrentAngle() - offsetRadArm) * arm.armLength;
    }

    public double getCurrentY(){
        return Math.sin(arm.armRotation.getCurrentAngle() - offsetRadArm) * arm.armLength + slides.getLength() + baseHeight;
    }

    public void moveToStart(){
        arm.setArmRotation(0.0, 1.0);
        slides.setTargetLength(0.0);
    }

    public void prepareTransfer() {
        state = State.TRANSFER_WAIT_ARM;
    }

    public void startTransfer() {
        state = State.TRANSFER_GRAB;
    }

    public boolean isSampleReady() {
        return state == State.HOLD;
    }

    public void startSampleDeposit() {
        state = State.SAMPLE_RAISE;
    }

    public void finishSampleDeposit() {
        state = State.SAMPLE_DEPOSIT;
    }

    public boolean isSampleDepositDone() {
        return state == State.IDLE;
    }

    public void startOuttake() {
        state = State.OUTTAKE_MOVE;
    }

    public boolean isOuttakeDone() {
        return state == State.HOLD;
    }

    public void grabSpecimen() {
        state = State.GRAB_MOVE;
    }

    public void finishSpecimenGrab() {
        state = State.GRAB;
    }

    public boolean isSpecimenReady() {
        return state == State.HOLD;
    }

    public void startSpecimenDeposit() {
        state = State.SPECI_RAISE;
    }

    public void finishSpecimenDeposit() {
        state = State.SPECI_DEPOSIT;
    }

    public boolean isSpecimenDepositDone() {
        return state == State.IDLE;
    }

    public void retract() {
        state = State.RETRACT;
    }

    public boolean isRetractDone(){
        return state == State.IDLE;
    }
}
