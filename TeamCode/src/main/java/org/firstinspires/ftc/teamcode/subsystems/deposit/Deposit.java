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
        SAMPLE_RAISE,
        SAMPLE_WAIT,
        SAMPLE_DEPOSIT,
        OUTTAKE_START,
        OUTTAKE_DROP,
        OUTTAKE_WAIT,
        SPECIMEN_GRAB_START,
        SPECIMEN_GRAB_WAIT,
        SPECIMEN_GRAB_CLOSE,
        SPECIMEN_GRAB_RETURN,
        SPECIMEN_RAISE,
        SPECIMEN_WAIT,
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

    private final double initX = 0.0, initY = 0.0, initArmAngle = Math.toRadians(60.0);
    private final double intakeX = 10.0, intakeY = -2.0, intakeWaitY = -1.0;
    private final double outtakeX = -11.816, outtakeY = 0.0, grabX = -5.0, grabY = -4.0;
    private final double sampleBasketX = -2.0, sampleBasketY = 46.0;
    private final double specimenBarX = 10.0, specimenBarY = 27.0;

    public Deposit(Robot robot){
        this.robot = robot;
        this.slides = new Slides(robot);

        arm = new Arm(robot);

        state = State.IDLE;
    }

    public void update(){
        if (Globals.TESTING_DISABLE_CONTROL && Globals.RUNMODE == RunMode.TESTER) return;
        arm.update();

        switch(state){
            case IDLE:
                break;
            case TRANSFER_PREPARE_1:
                setDepositPositions(intakeX, intakeWaitY);
                calculateMoveTo();

                arm.setDiffy(Math.toRadians(270.0), Math.toRadians(0.0));
                arm.openClaw();

                if(arm.checkReady()){
                    state = State.TRANSFER_PREPARE_2;
                }
                break;
            case TRANSFER_PREPARE_2:
                arm.setMgnPosition(moveToX);
                slides.setTargetLength(moveToY);
                arm.setArmAngle(moveToArmAngle);


                if(arm.checkReady() && slides.inPosition(0.5)){
                    state = State.TRANSFER_WAIT;
                }
                break;
            case TRANSFER_WAIT:
                break;
            case TRANSFER_GRAB_1:
                //robot.intake.transfer();

                if(robot.sensors.getIntakeColor() == Sensors.BlockColor.BLUE || robot.sensors.getIntakeColor() == Sensors.BlockColor.YELLOW){
                    state = State.TRANSFER_GRAB_2;
                }
                break;
            case TRANSFER_GRAB_2:
                setDepositPositions(intakeX, intakeY);
                calculateMoveTo();

                arm.setMgnPosition(moveToX);
                slides.setTargetLength(moveToY);
                arm.setArmAngle(moveToArmAngle);

                if(arm.checkReady() && slides.inPosition(0.5)){
                    state = State.TRANSFER_GRAB_3;
                }
                break;
            case TRANSFER_GRAB_3:
                arm.closeClaw();

                //TODO: How would I determine when I have actually grabbed it? I used the color sensor to determine when the robot sees the sample to start intake from there, but I'm not sure how to use it to determine grabbing
                if(arm.checkReady() && (robot.sensors.getIntakeColor() == Sensors.BlockColor.BLUE || robot.sensors.getIntakeColor() == Sensors.BlockColor.YELLOW)){
                    state = State.TRANSFER_END;
                }
                break;
            case TRANSFER_END:
                resetToStart();

                arm.setMgnPosition(moveToX);
                slides.setTargetLength(moveToY);
                arm.setArmAngle(moveToArmAngle);

                if(arm.checkReady() && slides.inPosition(0.5)){
                    state = State.READY;
                }
                break;
            case READY:
                break;
            case SAMPLE_RAISE:
                setDepositPositions(sampleBasketX, sampleBasketY);
                calculateMoveTo();

                arm.setMgnPosition(moveToX);
                slides.setTargetLength(moveToY);
                arm.setArmAngle(moveToArmAngle);
                arm.setDiffy(Math.toRadians(270.0), Math.toRadians(0.0));

                if(arm.checkReady() && slides.inPosition(0.5)){
                    state = State.SAMPLE_WAIT;
                }
                break;
            case SAMPLE_WAIT:
                break;
            case SAMPLE_DEPOSIT:
                arm.openClaw();

                if(arm.checkReady()){
                    state = State.RETRACT;
                }
                break;
            case OUTTAKE_START:
                setDepositPositions(outtakeX, outtakeY);
                calculateMoveTo();

                arm.setMgnPosition(moveToX);
                slides.setTargetLength(moveToY);
                arm.setArmAngle(moveToArmAngle);

                if(arm.checkReady() && slides.inPosition(0.5)){
                    state = State.OUTTAKE_DROP;
                }
                break;
            case OUTTAKE_DROP:
                arm.openClaw();

                if(arm.checkReady()){
                    state = State.OUTTAKE_WAIT;
                }
                break;
            case OUTTAKE_WAIT:
                break;
            case SPECIMEN_GRAB_START:
                setDepositPositions(grabX, grabY);
                calculateMoveTo();

                arm.setMgnPosition(moveToX);
                slides.setTargetLength(moveToY);
                arm.setArmAngle(moveToArmAngle);
                arm.setDiffy(Math.toRadians(180.0), Math.toRadians(90.0));
                arm.openClaw();

                if(arm.checkReady() && slides.inPosition(0.5)){
                    state = State.SPECIMEN_GRAB_WAIT;
                }
                break;
            case SPECIMEN_GRAB_WAIT:
                break;
            case SPECIMEN_GRAB_CLOSE:
                arm.closeClaw();

                if(arm.checkReady()){
                    state = State.SPECIMEN_GRAB_RETURN;
                }
                break;
            case SPECIMEN_GRAB_RETURN:
                resetToStart();

                arm.setMgnPosition(moveToX);
                slides.setTargetLength(moveToY);
                arm.setMgnPosition(moveToArmAngle);

                if(arm.checkReady()){
                    state = State.READY;
                }
                break;
            case SPECIMEN_RAISE:
                setDepositPositions(specimenBarX, specimenBarX);
                calculateMoveTo();

                arm.setMgnPosition(moveToX);
                slides.setTargetLength(moveToY);
                arm.setMgnPosition(moveToArmAngle);
                arm.setDiffy(Math.toRadians(0.0), Math.toRadians(180.0));

                if(arm.checkReady() && slides.inPosition(0.5)){
                    state = State.SPECIMEN_WAIT;
                }
                break;
            case SPECIMEN_WAIT:
                break;
            case SPECIMEN_DEPOSIT:
                //TODO: How do we determine shove positions again? I feel like having a constant value will be a bad idea, which sensor would this fall under? Or would it be driver control? If its the latter, tell me when the teleop functions is made
                setDepositPositions(specimenBarX, specimenBarY + 2.0);
                calculateMoveTo();

                arm.setMgnPosition(moveToX);
                slides.setTargetLength(moveToY);
                arm.setMgnPosition(moveToArmAngle);

                if(arm.checkReady() && slides.inPosition(0.5)){
                    state = State.RELEASE;
                }
                break;
            case RELEASE:
                arm.openClaw();

                if(arm.checkReady()){
                    state = State.RETRACT;
                }
                break;
            case RETRACT:
                resetToStart();

                arm.setMgnPosition(moveToX);
                slides.setTargetLength(moveToY);
                arm.setMgnPosition(moveToArmAngle);

                if(arm.checkReady() && slides.inPosition(0.5)){
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
        state = State.SAMPLE_RAISE;
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

    // The Robot FSM can call either grabSpecimen() or retract() after the RobotState.OUTTAKE
    public boolean isOuttakeDone() {
        return state == State.OUTTAKE_WAIT && robot.sensors.getIntakeColor() == Sensors.BlockColor.NONE;
    }

    public void grabSpecimen() {
        state = State.SPECIMEN_GRAB_START;
    }

    // NOTE: This method will be called rapidly if the driver holds the "grab specimen" button
    public void finishSpecimenGrab() {
        if(state == State.SPECIMEN_GRAB_WAIT && robot.sensors.getIntakeColor() == Sensors.BlockColor.BLUE) state = State.SPECIMEN_GRAB_CLOSE;
    }

    public boolean isSpecimenReady() {
        return state == State.READY;
    }

    public void startSpecimenDeposit() {
        state = State.SPECIMEN_RAISE;
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
        double baseX = arm.calcMgnPosition();
        double baseY = slides.getLength();

        //check if only horizontal movement is possible now to minimize slides usage
        if(Math.abs(baseY - targetY) < arm.getArmLength() && targetX >= 0.0){
            double pos1 = targetX - Math.sqrt(arm.getArmLength() * arm.getArmLength() - (baseY - targetY) * (baseY - targetY));
            double pos2 = targetX + Math.sqrt(arm.getArmLength() * arm.getArmLength() - (baseY - targetY) * (baseY - targetY));
            if(Math.abs(pos1 - baseX) < Math.abs(pos2 - baseX) && pos1 >= 0.0){
                moveToX = pos1;
            }else{
                moveToX = pos2;
            }
            moveToY = baseY;
            moveToArmAngle = Math.atan2(moveToY - targetY, moveToX - targetX);
        }else{
            double slope = (baseY - targetY)/(baseX - targetX);
            double pos1x = Utils.minMaxClip(targetX - Math.sqrt(arm.getArmLength() * arm.getArmLength()/(slope * slope + 1)), 0.0, 11.816);
            double pos1y = Utils.minMaxClip(slope * (pos1x - targetX) + targetY, 0.0, 50.0);
            double pos2x = Utils.minMaxClip(targetX + Math.sqrt(arm.getArmLength() * arm.getArmLength()/(slope * slope + 1)), 0.0, 11.816);
            double pos2y = Utils.minMaxClip(slope * (pos2x - targetX) + targetY, 0.0, 50.0);

            if(Math.abs(pos1y - targetY) < Math.abs(pos2y - targetY)){
                moveToX = pos1x;
                moveToY = pos1y;
            }else{
                moveToX = pos2x;
                moveToY = pos2y;
            }

            moveToArmAngle = Math.atan2(moveToY - targetY, moveToX - targetX);
        }
    }

    public void resetToStart(){
        moveToX = 0.0;
        moveToY = 0.0;
        moveToArmAngle = Math.toRadians(60.0);
    }
}
