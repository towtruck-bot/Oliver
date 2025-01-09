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

    private Robot robot;
    private Slides slides;
    private Arm arm;

    // x values are measured from the base of the arm
    // y values are measured from the ground
    // intake is positive x direction

    //TODO: Retune servos

    private final double baseHeight = 10.75; //TODO: Get more accurate measurement
    private final double intakeWaitX = 5.905314961 * Math.cos(Math.PI / 12), intakeWaitY = baseHeight + 5.905314961 * Math.sin(Math.PI / 12), intakeX = 5.905314961, intakeY = baseHeight; //TODO: Get more accurate intake locations
    private final double holdX = 5.905314961, holdY = 0.0;
    private final double sampleX = -1.3, sampleLY = 26.0, sampleHY = 44.6;
    private final double outtakeX = 5.905314961, outtakeY = baseHeight;
    private final double speciX = -5.9, speciLSY = 13.5, speciLEY = 15.4, speciHSY = 27.6, speciHEY = 29.6;

    public Deposit(Robot robot){
        this.robot = robot;
        this.slides = new Slides(robot);

        arm = new Arm(robot);

        state = Globals.hasSpecimenPreload ? State.HOLD : State.IDLE;
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
                arm.setClawRotation(arm.armRotation.getCurrentAngle() - Math.PI / 2.0, 1.0); //TODO: Fine tune this

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
                moveTo(sampleX, sampleHY);

                if(arm.inPosition() && slides.inPosition(0.5)){
                    state = State.SAMPLE_CLAW;
                }
                break;
            case SAMPLE_CLAW:
                arm.setClawRotation(Math.PI - arm.armRotation.getCurrentAngle(), 1.0);

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
                arm.setClawRotation(0, 1.0);
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
                arm.setClawRotation(Math.PI - arm.armRotation.getCurrentAngle(), 1.0);

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
    }

    //TODO: Presumes intake direction parallel to ground is 0 radians, need to figure out how deal with negative angle. offsets?
    public void moveTo(double targetX, double targetY){
        double armTargetRad = Math.acos(targetX/arm.armLength);
        double slidesTargetLength = Math.max(targetY - baseHeight - arm.armLength * Math.sin(armTargetRad), 0.0);

        arm.setArmRotation(armTargetRad, 1.0);
        slides.setTargetLength(slidesTargetLength);
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
