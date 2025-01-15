package org.firstinspires.ftc.teamcode.subsystems.deposit;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.utils.Globals;
import org.firstinspires.ftc.teamcode.utils.TelemetryUtil;

public class Deposit {
    public enum State{
        IDLE,
        TRANSFER_PREPARE,
        TRANSFER_WAIT,
        TRANSFER_GRAB,
        TRANSFER_CLOSE,
        TRANSFER_FINISH,
        HOLD,
        SAMPLE_RAISE,
        SAMPLE_WAIT,
        SAMPLE_DEPOSIT,
        OUTTAKE_MOVE,
        OUTTAKE_RELEASE,
        GRAB_MOVE,
        GRAB_WAIT,
        GRAB,
        GRAB_RETRACT,
        GRAB_HOLD,
        SPECI_RAISE,
        SPECI_WAIT,
        SPECI_DEPOSIT,
        RELEASE,
        RETRACT
    };

    public State state;

    private final Robot robot;
    public final Slides slides;
    public final Arm arm;

    // x values are measured from the base of the arm
    // y values are measured from the ground
    // intake is positive x direction

    private final double baseHeight = 10.75;

    private final double intakeWaitRad = Math.PI / 12, intakeWaitY = 0.0, intakeRad = 0.0, intakeY = 0.0, intakeClawRad = -2.3;
    private final double holdRad = 0.0, holdY = 0.0, holdClawRad = 2.0, holdGrabRad = -0.3;
    private final double sampleHY = 33.85, sampleRad = Math.atan2(sampleHY, -1.3);
    // sampleLY = 15.25,
    private final double outtakeRad = Math.PI, outtakeY = 0.0, outtakeReleaseRad = 0.0, grabRad = 2.0;
    private final double  speciHRad = 1.8, speciHClawRad = 0.7, speciHSY = 15.0, speciHEY = 18.0;
    // speciLRad = 2.75, speciLClawRad = 0.0, speciLSY = 0.0, speciLEY = 0.0,
    private double targetY = 0.0;

    public Deposit(Robot robot){
        this.robot = robot;
        this.slides = new Slides(this.robot);

        arm = new Arm(this.robot);

        state = Globals.hasSpecimenPreload ? State.HOLD : State.RETRACT;
    }

    public void update(){
        switch(state){
            case IDLE:
                break;
            case TRANSFER_PREPARE:
                moveToWithRad(intakeWaitRad, intakeWaitY);
                arm.setClawRotation(intakeClawRad, 1.0);
                arm.sampleOpen();

                if(arm.inPosition() && slides.inPosition(0.8) && arm.clawFinished()){
                    state = State.TRANSFER_WAIT;
                }
                break;
            case TRANSFER_WAIT:
                moveToWithRad(intakeWaitRad, intakeWaitY);
                break;
            case TRANSFER_GRAB:
                moveToWithRad(intakeRad, intakeY);

                if(arm.inPosition() && slides.inPosition(0.8)){
                    state = State.TRANSFER_CLOSE;
                }
                break;
            case TRANSFER_CLOSE:
                moveToWithRad(intakeRad, intakeY);
                arm.sampleClose();

                if(arm.clawFinished() && intakeDone){
                    intakeDone = false;
                    state = State.TRANSFER_FINISH;
                }
                break;
            case TRANSFER_FINISH:
                moveToWithRad(holdRad, holdY);
                arm.clawRotation.setTargetAngle(holdClawRad, 1.0);

                if(arm.inPosition()){
                    state = State.HOLD;
                }
                break;
            case HOLD:
                moveToWithRad(holdRad, holdY);
                break;
            case SAMPLE_RAISE:
                moveToWithRad(sampleRad, sampleHY);
//                arm.setClawRotation(Math.PI - (arm.armRotation.getCurrentAngle() - offsetRadArm) + offsetRadClaw, 1.0);

                if(arm.inPosition() && slides.inPosition(0.8)){
                    state = State.SAMPLE_WAIT;
                }
                break;
            case SAMPLE_WAIT:
                moveToWithRad(sampleRad, sampleHY);
                break;
            case SAMPLE_DEPOSIT:
                moveToWithRad(sampleRad, sampleHY);
                arm.sampleOpen();

                if(arm.clawFinished()){
                    state = State.RETRACT;
                }
                break;
            case OUTTAKE_MOVE:
                moveToWithRad(outtakeRad, outtakeY);
                arm.clawRotation.setTargetAngle(outtakeReleaseRad, 1.0);

                if(arm.inPosition() && slides.inPosition(0.8)){
                    state = State.OUTTAKE_RELEASE;
                }
                break;
            case OUTTAKE_RELEASE:
                moveToWithRad(outtakeRad, outtakeY);
                arm.sampleOpen();

                if(arm.clawFinished()){
                    state = State.RETRACT;
                }
                break;
            case GRAB_MOVE:
                moveToWithCoord(holdRad, holdY);
                arm.setClawRotation(holdGrabRad, 1.0);
                arm.speciOpen();

                if(arm.inPosition() && slides.inPosition(0.8) && arm.clawFinished()){
                    state = State.GRAB_WAIT;
                }
                break;
            case GRAB_WAIT:
                moveToWithCoord(holdRad, holdY);
                break;
            case GRAB:
                moveToWithCoord(holdRad, holdY);
                arm.speciClose();

                if(arm.clawFinished()){
                    state = State.GRAB_RETRACT;
                }
                break;
            case GRAB_RETRACT:
                moveToWithRad(grabRad, holdY);
                arm.setClawRotation(speciHClawRad, 1.0);

                if(arm.inPosition()){
                    state = State.GRAB_HOLD;
                }
                break;
            case GRAB_HOLD:
                moveToWithRad(grabRad, holdY);
                arm.setClawRotation(speciHClawRad, 1.0);
                break;
            case SPECI_RAISE:
                moveToWithRad(speciHRad, targetY);

                if(arm.inPosition() && slides.inPosition(0.8)){
                    state = State.SPECI_WAIT;
                }
                break;
            case SPECI_WAIT:
                moveToWithRad(speciHRad, targetY);
                break;
            case SPECI_DEPOSIT:
                moveToWithRad(speciHRad, targetY);

                if(arm.inPosition() && slides.inPosition(0.8)){
                    state = State.RELEASE;
                }
                break;
            case RELEASE:
                moveToWithRad(speciHRad, slides.getLength());
                arm.speciOpen();

                if(arm.clawFinished()){
                    state = State.RETRACT;
                }
                break;
            case RETRACT:
                moveToStart();

                if(arm.inPosition() && slides.inPosition(0.8)){
                    state = State.IDLE;
                }
                break;
        }

        slides.update();

        TelemetryUtil.packet.put("Deposit:: Current State", state);
        TelemetryUtil.packet.put("Deposit:: Arm inPosition()", arm.inPosition());
        TelemetryUtil.packet.put("Deposit:: Slides inPosition()", slides.inPosition(0.5));
        TelemetryUtil.packet.put("Deposit:: Claw inPosition()", arm.clawFinished());
    }

    public void moveToWithCoord(double targetX, double targetY){
        double armTargetRad = Math.acos(targetX/arm.armLength) * (targetY >= baseHeight ? 1 : -1);
        double slidesTargetLength = Math.max(targetY - (baseHeight + arm.armLength * Math.sin(armTargetRad)), 0.0);

        arm.setArmRotation(armTargetRad, 1.0);
        slides.setTargetLength(slidesTargetLength);
    }

    public void moveToWithRad(double armTargetRad, double targetY){
        arm.setArmRotation(armTargetRad, 1.0);
        slides.setTargetLength(targetY);

        TelemetryUtil.packet.put("Deposit:: Actual Height", targetY + Math.sin(arm.armRotation.getCurrentAngle()) * arm.armLength + baseHeight);
        TelemetryUtil.packet.put("Deposit:: Slides Height", targetY);
        TelemetryUtil.packet.put("Deposit:: Arm Height", Math.sin(arm.armRotation.getCurrentAngle()) * arm.armLength);
        TelemetryUtil.packet.put("Deposit:: Arm Angle", armTargetRad);
    }

    private boolean intakeDone = false;
    public void intakeDone() {
        intakeDone = true;
    }

    public void setDepositHeight(double targetY){
        this.targetY = targetY;
    }

    public void moveToStart(){
        arm.setArmRotation(0.0, 1.0);
        slides.setTargetLength(0.0);
    }

    public void prepareTransfer() {
        state = State.TRANSFER_PREPARE;
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
