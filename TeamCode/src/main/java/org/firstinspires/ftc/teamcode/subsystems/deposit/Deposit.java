package org.firstinspires.ftc.teamcode.subsystems.deposit;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.utils.Utils;

public class Deposit {
    public enum State{
        IDLE,
        TRANSFER_START,
        TRANSFER_END,
        TRANSFER_WAIT,
        TRANSFER_GRAB,
        READY,
        SAMPLE_RAISE,
        SAMPLE_DEPOSIT,
        OUTTAKE_MOVE,
        OUTTAKE_RELEASE,
        GRAB_SET,
        GRAB,
        GRAB_RETRACT,
        SPECIMEN_RAISE,
        SPECIMEN_DEPOSIT,
        RELEASE,
        BUFFER,
        RETRACT
    };
    public State state;

    public Robot robot;
    public Slides slides;
    public Arm arm;

    // TODO: Servos have mechanically set 0 positions; make sure mechanical sets 0's accurately and defines max ranges. be careful of priority servo reverse, may not work - profe

    private double currX, currY, currArmAngle;
    private double targetX, targetY;
    private double moveToX, moveToY, moveToArmAngle;
    private boolean tooClose = false;

    private final double initX = 0.0, initY = arm.getArmLength();
    private final double intakeWaitY = -1.0, intakeX = 10.0, intakeY = -2.0; // TODO: Update these values
    private final double sampleBasketX = -2.0, sampleBasketY = 46.0;
    private final double specimenBarX = 10.0, specimenBarY = 27.0;
    private final double outtakeX = -11.816, outtakeY = 0.0, grabX = -5.0, grabY = -4.0;

    public Deposit(Robot robot){
        this.robot = robot;
        this.slides = new Slides(robot);

        arm = new Arm(robot);

        //TODO: Determine correct starting values
        this.currX = initX;
        this.currY = initY;

        state = State.IDLE;
    }

    //TODO: V4 bar keeps everything attached parallel to the ground, i.e. when arm moves to 45 claw is still parallel don't be a monkey - james
    public void update(){
        switch(state){
            //Deposit remains motionless
            case IDLE:
                break;
            //Begin transfer: move arm to 1 inch above transfer basket in two steps
            case TRANSFER_START:
                setDepositPositions(intakeX + Utils.minMaxClip(robot.sensors.getIntakeExtensionPosition(), 0.0, 2.0), intakeWaitY);
                calculateMoveTo();

                //Math used to determine if the arm is too close the transfer-wait location, take two different paths to prevent hitting parts of robot
                if((targetX - currX) * (targetX - currX) + (targetY - currY) * (targetY - currY) <= arm.getArmLength() * arm.getArmLength()){
                    tooClose = true;
                    arm.setMgnPosition(moveToX);
                    slides.setTargetLength(moveToY);
                }else{
                    tooClose = false;
                    arm.setArmAngle(moveToArmAngle);
                }

                if(arm.checkReady()){
                    state = State.TRANSFER_END;
                }
                break;
            //Transfer step 2
            case TRANSFER_END:
                if(tooClose){
                    arm.setArmAngle(moveToArmAngle);
                }else{
                    arm.setMgnPosition(moveToX);
                    slides.setTargetLength(moveToY);
                }

                if(arm.checkReady() && slides.inPosition(0.5)){
                    state = State.TRANSFER_WAIT;
                    currX = moveToX;
                    currY = moveToY;
                    currArmAngle = moveToArmAngle;
                }
                break;
            //Wait above the transfer bin, orient claw correctly
            //TODO: Determine orientation of sample in transfer basket
            case TRANSFER_WAIT:
                arm.setDiffy(Math.toRadians(-90.0) - moveToArmAngle, Math.toRadians(90.0));
                arm.openClaw();
                if(arm.checkReady() && robot.sensors.getIntakeExtensionPosition() == 0.0){
                    state = State.TRANSFER_GRAB;
                }
                break;
            //Actually grab the sample from basket
            case TRANSFER_GRAB:
                setDepositPositions(intakeX, intakeY);
                calculateMoveTo();

                arm.setMgnPosition(moveToX);
                arm.setArmAngle(moveToArmAngle);
                slides.setTargetLength(moveToY);

                if(arm.checkReady()){
                    arm.closeClaw();
                    state = State.READY;
                }
                break;
            //Motionless state similar to idle; used to indicate carrying a sample
            case READY:
                //TODO: Is there a sensor of sort on the claw to confirm I've grabbed the sample?
                break;
            //Raise arm with sample to deposit location. Arm freezes at top so robot can be moved slightly to adjust position
            case SAMPLE_RAISE:
                setDepositPositions(sampleBasketX, sampleBasketY);
                calculateMoveTo();

                arm.setMgnPosition(moveToX);
                slides.setTargetLength(moveToY);
                arm.setArmAngle(moveToArmAngle);

                if(arm.checkReady() && slides.inPosition(0.5)){
                    state = State.SAMPLE_DEPOSIT;
                    currX = moveToX;
                    currY = moveToY;
                    currArmAngle = moveToArmAngle;
                }
                break;
            //Actually drop the sample into the target basket
            case SAMPLE_DEPOSIT:
                arm.openClaw();
                if(arm.checkReady()){
                    //Goes to buffer state so moving arm + slides at the same time will not hit field
                    state = State.BUFFER;
                }
                break;
            //During Teleop, start from this state to outtake the sample. Current state will move the grabbed sample to behind the robot/drop location
            case OUTTAKE_MOVE:
                setDepositPositions(outtakeX, outtakeY);
                calculateMoveTo();

                arm.setMgnPosition(moveToX);
                slides.setTargetLength(moveToY);
                arm.setArmAngle(moveToArmAngle);
                arm.setDiffy(Math.toRadians(180.0), Math.toRadians(90.0));

                if(arm.checkReady()){
                    state = State.OUTTAKE_RELEASE;
                }
                break;
            //Actually drop the sample at assembly location
            case OUTTAKE_RELEASE:
                arm.openClaw();
                if(arm.checkReady()){
                    //Go to retract so specimen cycle can resume
                    state = State.RETRACT;
                }
                break;
            //Move the claw to pick up location, wait for robot/driver to make adjustments
            case GRAB_SET:
                setDepositPositions(grabX, grabY);
                calculateMoveTo();

                arm.setMgnPosition(moveToX);
                slides.setTargetLength(moveToY);
                arm.setArmAngle(moveToArmAngle);

                arm.setDiffy(Math.toRadians(180.0), Math.toRadians(90.0));
                arm.openClaw();

                //TODO: Require a sensors check if the block is in position? Or at driver discretion
                if(arm.checkReady() && slides.inPosition(0.5)){
                    state = State.GRAB;
                }
                break;
            //Actually grab the now-specimen
            case GRAB:
                arm.closeClaw();

                if(arm.checkReady()){
                    //Separate grab Retract that will make it easier for the the robot to move around with the specimen
                    state = State.GRAB_RETRACT;
                }
                break;
            //A hold-state of sorts, retracts the claw + specimen to allow for easier transportation
            case GRAB_RETRACT:
                //TODO: Same as RETRACT State
                setDepositPositions(0.0, arm.getArmLength());
                calculateMoveTo();

                arm.setMgnPosition(moveToX);
                slides.setTargetLength(moveToY);
                arm.setArmAngle(moveToArmAngle);

                if(arm.checkReady()){
                    state = State.READY;
                }
                break;
            //Raise claw + specimen to a height just below the deposit-bar to allow for robot/driver adjustments
            case SPECIMEN_RAISE:
                setDepositPositions(specimenBarX, specimenBarY - 2.0);
                calculateMoveTo();

                arm.setMgnPosition(moveToX);
                slides.setTargetLength(moveToY);
                arm.setArmAngle(moveToArmAngle);
                arm.setDiffy(0.0, Math.toRadians(180.0));

                if(arm.checkReady() && slides.inPosition(0.5)){
                    state = State.SPECIMEN_DEPOSIT;
                    currX = moveToX;
                    currY = moveToY;
                    currArmAngle = moveToArmAngle;
                }
                break;
            //Actually attach the specimen to the deposit bar
            case SPECIMEN_DEPOSIT:
                setDepositPositions(specimenBarX, specimenBarY);
                calculateMoveTo();

                arm.setMgnPosition(moveToX);
                slides.setTargetLength(moveToY);
                arm.setArmAngle(moveToArmAngle);

                if(arm.checkReady() && slides.inPosition(0.5)){
                    //Goes to RELEASE state to let go of the specimen
                    //TODO: add a case or if statement that will determine if specimen clip actually occurred? or at least do not go directly to release
                    state = State.RELEASE;
                    currX = moveToX;
                    currY = moveToY;
                    currArmAngle = moveToArmAngle;
                }
                break;
            //Actually let go of specimen
            case RELEASE:
                arm.openClaw();
                arm.setDiffy(0.0, 0.0);
                if(arm.checkReady()){
                    //Goes to buffer state so moving arm + slides at the same time will not hit field
                    state = State.BUFFER;
                }
                break;
            //Depending on direction of arm(can determine where robot is depositing from that), move toward (0,0) by 1 inch in x and y so buffer distance is created as to not hit field
            case BUFFER:
                //facing back/deposit side : facing front/intake side
                setDepositPositions(currX + (currArmAngle > Math.toRadians(90.0) ? 1.0 : -1.0), currY - 1.0);
                calculateMoveTo();

                arm.setMgnPosition(moveToX);
                slides.setTargetLength(moveToY);
                arm.setArmAngle(moveToArmAngle);

                if(arm.checkReady() && slides.inPosition(0.5)){
                    state = State.RETRACT;
                    currX = moveToX;
                    currY = moveToY;
                    currArmAngle = moveToArmAngle;
                }
                break;
            //Move arm and slides to initial position, prepare for next round
            case RETRACT:
                //TODO: May need a separate reset function as setDepositPositions would put claw in this spot
                setDepositPositions(initX, initY);
                calculateMoveTo();

                arm.setMgnPosition(moveToX);
                slides.setTargetLength(moveToY);
                arm.setArmAngle(moveToArmAngle);
                arm.openClaw();

                if(arm.checkReady() && slides.inPosition(0.5)){
                    //Goes back to IDLE to wait for next command
                    state = State.IDLE;
                    currX = moveToX;
                    currY = moveToY;
                    currArmAngle = moveToArmAngle;
                }
                break;
        }
    }

    public void startTransfer() {
        state = State.TRANSFER_START;
    }

    public boolean isSampleReady() {
        return state == State.READY;
    }

    public void startSampleDeposit() {
        state = State.SAMPLE_RAISE;
    }

    // TODO: Drop sample in bucket and return to idle state
    public void finishSampleDeposit() {
        state = State.SAMPLE_DEPOSIT;
    }

    public boolean isSampleDepositDone() {
        return state == State.IDLE;
    }

    public void startOuttake() {
        state = State.OUTTAKE_MOVE;
    }

    // TODO: isOuttakeDone should check if the claw dropped the block outside
    // The Robot FSM can call either grabSpecimen() or retract() after the RobotState.OUTTAKE
    public boolean isOuttakeDone() {
        return state == State.IDLE;
    }

    public void grabSpecimen() {
        state = State.GRAB_SET;
    }

    // TODO: IF the claw is down and has a specimen in it, close the claw and move to specimen ready
    // NOTE: This method will be called rapidly if the driver holds the "grab specimen" button
    public void finishSpecimenGrab() {
        state = State.GRAB;
    }

    public boolean isSpecimenReady() {
        return state == State.READY;
    }

    public void startSpecimenDeposit() {
        state = State.SPECIMEN_RAISE;
    }

    // TODO: Hook specimen on rod and return to idle state
    public void finishSpecimenDeposit() {
        state = State.SPECIMEN_DEPOSIT;
    }

    public boolean isSpecimenDepositDone() {
        return state == State.IDLE;
    }

    // TODO: Go back to idle state
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
}
