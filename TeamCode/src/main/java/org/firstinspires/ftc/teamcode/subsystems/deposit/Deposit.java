package org.firstinspires.ftc.teamcode.subsystems.deposit;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.sensors.Sensors;
import org.firstinspires.ftc.teamcode.utils.Globals;
import org.firstinspires.ftc.teamcode.utils.RunMode;
import org.firstinspires.ftc.teamcode.utils.Utils;

public class Deposit {
    public enum State{
        IDLE,
        TRANSFER_PREPARE_1,
        TRANSFER_PREPARE_2,
        TRANSFER_WAIT,
        TRANSFER_GRAB_1,
        TRANSFER_GRAB_2,
        TRANSFER_GRAB_3,
        TRANSFER_END,
        READY,
        SAMPLE_RAISE_1,
        SAMPLE_RAISE_2,
        SAMPLE_WAIT,
        SAMPLE_DEPOSIT,
        OUTTAKE_START,
        OUTTAKE_DROP,
        OUTTAKE_WAIT,
        SPECIMEN_GRAB_START_1,
        SPECIMEN_GRAB_START_2,
        SPECIMEN_GRAB_WAIT,
        SPECIMEN_GRAB_CLOSE,
        SPECIMEN_GRAB_RETURN,
        HOLD,
        SPECIMEN_RAISE_1,
        SPECIMEN_RAISE_2,
        SPECIMEN_RAISE_3,
        SPECIMEN_RAISE_WAIT,
        SPECIMEN_DEPOSIT,
        RELEASE,
        RETRACT
    };
    public State state;

    public Robot robot;
    public Slides slides;
    public Arm arm;

    // TODO: Servos have mechanically set 0 positions; make sure mechanical sets 0's accurately and defines max ranges. be careful of priority servo reverse, may not work - profe

    private double targetX, targetY;
    private double moveToX, moveToY, moveToArmAngle;

    //TODO: Verify All Values. Important!!!! LM2 robot all diff values
    //TODO: FOR X COORDINATES, REMEMBER TO ACCOUNT FOR CLAW LENGTH
    private final double transferPrepareX = 7.0677, transferPrepareY = 3.0653, transferX = 7.9256, transferY = 0.0, transferRad = 1.406;
    private final double sampleBasketX = 1.0, sampleBasketY = 1.0;
    private final double outtakeX = 1.0, outtakeY = 1.0, grabX = 1.0, grabY = 1.0;
    private final double movingX = 1.0, movingY = 1.0, movingClawRad = Math.PI / 2;
    private final double specimenBarOutsideX = 1.0, specimenBarInsideX = 1.0, specimenBarAtX = 1.0, specimenUnderBarY = 1.0, specimenAtBarY = 1.0;

    public Deposit(Robot robot){
        this.robot = robot;
        this.slides = new Slides(robot);

        arm = new Arm(robot);

        state = Globals.hasSpecimenPreload ? State.HOLD : State.IDLE;
    }

    public void update(){
        if (Globals.TESTING_DISABLE_CONTROL && Globals.RUNMODE == RunMode.TESTER) return;

        switch(state){
            case IDLE:
                resetToStart();
                updatePositions();

                break;
            case TRANSFER_PREPARE_1:
                setDepositPositions(transferPrepareX, transferPrepareY);
                calculateMoveTo();

                updatePositions();

                if(arm.inPosition() && slides.inPosition(0.5)){
                    state = State.TRANSFER_PREPARE_2;
                }
                break;
            case TRANSFER_PREPARE_2:
                arm.setClawRotation(transferRad, 1.0);
                arm.setClawSamplePrepare();

                if(arm.inPosition()){
                    state = State.TRANSFER_WAIT;
                }
                break;
            case TRANSFER_WAIT:
                break;
            case TRANSFER_GRAB_1:
                setDepositPositions(transferX, transferY);
                calculateMoveTo();

                updatePositions();

                if(arm.inPosition() && slides.inPosition(0.5)){
                    state = State.TRANSFER_GRAB_3;
                }
                break;
            case TRANSFER_GRAB_2:
                arm.setClawSampleGrab();

                if(arm.inPosition()){
                    state = State.TRANSFER_END;
                }
                break;
            case TRANSFER_GRAB_3:
                robot.intake.transfer();

                //No if - statement? because intake backspin should happen at the same time as claw moves up slightly, which is TRANSFER_END state -- James
                state = State.TRANSFER_END;
                break;
            case TRANSFER_END:
                setDepositPositions(0.0, arm.armLength);
                calculateMoveTo();

                updatePositions();

                if(arm.inPosition() && slides.inPosition(0.5)){
                    state = State.READY;
                }
                break;
            case READY:
                break;
            case SAMPLE_RAISE_1:
                setDepositPositions(sampleBasketX, sampleBasketY);
                calculateMoveTo();

                updatePositions();

                if(arm.inPosition() && slides.inPosition(0.5)){
                    state = State.SAMPLE_RAISE_2;
                }
                break;
            case SAMPLE_RAISE_2:
                arm.setClawRotation(Math.PI * 3.0/2.0, 1.0);

                if(arm.inPosition()){
                    state = State.SAMPLE_WAIT;
                }
                break;
            case SAMPLE_WAIT:
                break;
            case SAMPLE_DEPOSIT:
                arm.setClawSamplePrepare();

                if(arm.inPosition()){
                    state = State.RETRACT;
                }
                break;
            case OUTTAKE_START:
                setDepositPositions(outtakeX, outtakeY);
                calculateMoveTo();

                updatePositions();

                arm.setClawRotation(Math.PI, 1.0);

                if(arm.inPosition() && slides.inPosition(0.5)){
                    state = State.OUTTAKE_DROP;
                }
                break;
            case OUTTAKE_DROP:
                arm.setClawSamplePrepare();

                if(arm.inPosition()){
                    state = State.OUTTAKE_WAIT;
                }
                break;
            case OUTTAKE_WAIT:
                break;
            case SPECIMEN_GRAB_START_1:
                arm.setClawRotation(Math.PI, 1.0);
                arm.setClawSpeciPrepare();

                if(arm.inPosition()){
                    state = State.SPECIMEN_GRAB_START_2;
                }
                break;
            case SPECIMEN_GRAB_START_2:
                setDepositPositions(grabX, grabY);
                calculateMoveTo();

                updatePositions();

                if(arm.inPosition() && slides.inPosition(0.5)){
                    state = State.SPECIMEN_GRAB_WAIT;
                }
                break;
            case SPECIMEN_GRAB_WAIT:
                break;
            case SPECIMEN_GRAB_CLOSE:
                arm.setClawSpeciGrab();

                if(arm.inPosition()){
                    state = State.SPECIMEN_GRAB_RETURN;
                }
                break;
            case SPECIMEN_GRAB_RETURN:
                setDepositPositions(movingX, movingY);
                calculateMoveTo();

                updatePositions();

                arm.setClawRotation(movingClawRad, 1.0);

                if(arm.inPosition() && slides.inPosition(0.5)){
                    state = State.HOLD;
                }
                break;
            case HOLD:
                setDepositPositions(movingX, movingY);
                calculateMoveTo();

                updatePositions();
                break;
            case SPECIMEN_RAISE_1:
                setDepositPositions(specimenBarOutsideX, specimenUnderBarY);
                calculateMoveTo();

                updatePositions();

                if(arm.inPosition() && slides.inPosition(0.5)){
                    state = State.SPECIMEN_RAISE_2;
                }
                break;
            case SPECIMEN_RAISE_2:
                //TODO: specimenBarInsideX, prob gonna be difficult to determine exactly how far in to extend. Im thinking maybe 5 inches buffer? -- James
                setDepositPositions(specimenBarInsideX, specimenUnderBarY);
                calculateMoveTo();

                updatePositions();

                if(arm.inPosition()){
                    state = State.SPECIMEN_RAISE_3;
                }
                break;
            case SPECIMEN_RAISE_3:
                setDepositPositions(specimenBarInsideX, specimenAtBarY);
                calculateMoveTo();

                updatePositions();

                if(slides.inPosition(0.5)){
                    //state = State.SPECIMEN_RAISE_WAIT;
                    state = State.SPECIMEN_DEPOSIT;
                }
                break;
            case SPECIMEN_RAISE_WAIT:
                break;
            case SPECIMEN_DEPOSIT:
                //TODO: Talked with Neil and he will move the drivetrain back instead of us moving the horizontal rail backwards because we can't accurately determine a way to know how far the bar is from the clip(yet). -- James
//                setDepositPositions(specimenBarAtX, specimenAtBarY);
//                calculateMoveTo();
//
//                updatePositions();
//
//                if(arm.inPosition()){
//                    state = State.RELEASE;
//                }
                break;
            case RELEASE:
                arm.setClawSpeciPrepare();

                if(arm.inPosition()){
                    state = State.RETRACT;
                }
                break;
            case RETRACT:
                resetToStart();

                updatePositions();

                if(arm.inPosition() && slides.inPosition(0.5)){
                    state = State.IDLE;
                }
                break;
        }
    }

    public void prepareTransfer() {
        state = State.TRANSFER_PREPARE_1;
    }

    public void startTransfer() {
        state = State.TRANSFER_GRAB_1;
    }

    public boolean isSampleReady() {
        return state == State.READY;
    }

    public void startSampleDeposit() {
        state = State.SAMPLE_RAISE_1;
    }

    public void finishSampleDeposit() {
        state = State.SAMPLE_DEPOSIT;
    }

    public boolean isSampleDepositDone() {
        return state == State.IDLE;
    }

    public void startOuttake() {
        state = State.OUTTAKE_START;
    }

    public boolean isOuttakeDone() {
        return state == State.OUTTAKE_WAIT;
    }

    public void grabSpecimen() {
        state = State.SPECIMEN_GRAB_START_1;
    }

    public void finishSpecimenGrab() {
        state = State.SPECIMEN_GRAB_CLOSE;
    }

    public boolean isSpecimenReady() {
        return state == State.HOLD;
    }

    public void startSpecimenDeposit() {
        state = State.SPECIMEN_RAISE_1;
    }

    public void finishSpecimenDeposit() {
        state = State.SPECIMEN_DEPOSIT;
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

    public void setDepositPositions(double x, double y){
        targetX = x;
        targetY = y;
    }

    public void calculateMoveTo(){
        double baseX = arm.getHorizontalPos();
        //double baseY = slides.getLength();
        double baseY = 0.0;

        //isolate math section that assumes you cant move Y
        double pos1 = targetX - Math.sqrt(arm.armLength * arm.armLength - (baseY - targetY) * (baseY - targetY));
        double pos2 = targetX + Math.sqrt(arm.armLength * arm.armLength - (baseY - targetY) * (baseY - targetY));
        if(Math.abs(pos1 - baseX) < Math.abs(pos2 - baseX) && pos1 >= 0.0){
            moveToX = pos1;
        }else{
            moveToX = pos2;
        }
        moveToY = baseY;
        moveToArmAngle = Math.atan2(moveToY - targetY, moveToX - targetX);

//        //check if only horizontal movement is possible now to minimize slides usage
//        if(Math.abs(baseY - targetY) < arm.armLength && targetX >= 0.0){
//            double pos1 = targetX - Math.sqrt(arm.armLength * arm.armLength - (baseY - targetY) * (baseY - targetY));
//            double pos2 = targetX + Math.sqrt(arm.armLength * arm.armLength - (baseY - targetY) * (baseY - targetY));
//            if(Math.abs(pos1 - baseX) < Math.abs(pos2 - baseX) && pos1 >= 0.0){
//                moveToX = pos1;
//            }else{
//                moveToX = pos2;
//            }
//            moveToY = baseY;
//            moveToArmAngle = Math.atan2(moveToY - targetY, moveToX - targetX);
//        }else{
//            double slope = (baseY - targetY)/(baseX - targetX);
//            double pos1x = Utils.minMaxClip(targetX - Math.sqrt(arm.armLength * arm.armLength/(slope * slope + 1)), 0.0, 11.816);
//            double pos1y = Utils.minMaxClip(slope * (pos1x - targetX) + targetY, 0.0, 50.0);
//            double pos2x = Utils.minMaxClip(targetX + Math.sqrt(arm.armLength * arm.armLength/(slope * slope + 1)), 0.0, 11.816);
//            double pos2y = Utils.minMaxClip(slope * (pos2x - targetX) + targetY, 0.0, 50.0);
//
//            if(Math.abs(pos1y - targetY) < Math.abs(pos2y - targetY)){
//                moveToX = pos1x;
//                moveToY = pos1y;
//            }else{
//                moveToX = pos2x;
//                moveToY = pos2y;
//            }
//
//            moveToArmAngle = Math.atan2(moveToY - targetY, moveToX - targetX);
//        }
    }

    public void updatePositions(){
        arm.setHorizontalPos(moveToX, 1.0);
        slides.setTargetLength(moveToY);
        arm.setArmRotation(moveToArmAngle, 1.0);
    }

    public void resetToStart(){
        moveToX = 0.0;
        moveToY = 0.0;
        moveToArmAngle = Math.toRadians(60.0);
    }
}
