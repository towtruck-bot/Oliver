package org.firstinspires.ftc.teamcode.subsystems.deposit;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.sensors.Sensors;
import org.firstinspires.ftc.teamcode.utils.Utils;

public class Deposit {
    public enum State{
        IDLE,
        TRANSFER_START,
        TRANSFER_END,
        READY,
        SAMPLE_RAISE,
        SAMPLE_DEPOSIT,
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
    public Sensors sensors;

    //TODO: Servos have mechanically set 0 positions, make sure nathan xie does it so i can make him give me good zeros
    //TODO: also for max ranges too
    //TODO: make sure axons are in the range i want it to be in
    //TODO: priority servo reverse might not work, very sketchy thing, be careful of that function - profe
    private double currX, currY, currArmAngle;
    private double targetX, targetY;
    private double moveToX, moveToY, moveToArmAngle;
    private boolean tooClose = false;

    private final double intakeX = 10.0, intakeY = -2.0; // TODO: Update these values
    private final double sampleBasketX = -2.0, sampleBasketY = 46.0;
    private final double specimenBarX = 10.0, specimenBarY = 27.0;
    private final double grabX = -5.0, grabY = -4.0;

    public Deposit(Robot robot){
        this.robot = robot;
        this.sensors = robot.sensors;
        this.slides = new Slides(robot);

        arm = new Arm(robot);

        //TODO: Determine correct starting values
        this.currX = 0.0;
        this.currY = arm.getArmLength();

        state = State.IDLE;
    }

    //TODO: V4 bar keeps everything attached parallel to the ground, i.e. when arm moves to 45 claw is still parallel don't be a monkey - james
    public void update(){
        switch(state){
            case IDLE:
                break;
            case TRANSFER_START:
                setDepositPositions(intakeX, intakeY);
                calculateMoveTo();

                //TODO: prob shouldnt be actual numbers? depends on angle of approach
                arm.setDiffy(Math.toRadians(45.0), Math.toRadians(90.0));
                arm.openClaw();

                if((targetX - currX) * (targetX - currX) + (targetY - currY) * (targetY - currY) <= arm.getArmLength() * arm.getArmLength()){
                    tooClose = true;
                    arm.setMgnPosition(moveToX);
                    slides.setTargetLength(moveToY);
                }else{
                    arm.setArmAngle(moveToArmAngle);
                }

                if(arm.checkReady()){
                    state = State.TRANSFER_END;
                }
                break;
            case TRANSFER_END:
                if(tooClose){
                    arm.setArmAngle(moveToArmAngle);
                }else{
                    arm.setMgnPosition(moveToX);
                    slides.setTargetLength(moveToY);
                }

                if(arm.checkReady() && slides.inPosition(0.5)){
                    state = State.READY;
                    currX = moveToX;
                    currY = moveToY;
                    currArmAngle = moveToArmAngle;
                    arm.closeClaw();
                }
                break;
            case READY:
                //should i add anything here in particular for hold? i just want it to be state where the sample/speci is held
                //and waiting to be sent into a RAISE state
                break;
            case SAMPLE_RAISE:
                setDepositPositions(sampleBasketX, sampleBasketY + 2.0);
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
            case SAMPLE_DEPOSIT:
                arm.openClaw();
                if(arm.checkReady()){
                    state = State.BUFFER;
                }
                break;
            case GRAB_SET:
                setDepositPositions(grabX, grabY);
                calculateMoveTo();

                arm.setMgnPosition(moveToX);
                slides.setTargetLength(moveToY);
                arm.setArmAngle(moveToArmAngle);

                arm.setDiffy(Math.toRadians(180.0), Math.toRadians(90.0));
                arm.openClaw();

                if(arm.checkReady() && slides.inPosition(0.5)){
                    state = State.GRAB;
                }
                break;
            case GRAB:
                arm.closeClaw();

                if(arm.checkReady()){
                    state = State.GRAB_RETRACT;
                }
                break;
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
            case SPECIMEN_DEPOSIT:
                setDepositPositions(specimenBarX, specimenBarY);
                calculateMoveTo();

                arm.setMgnPosition(moveToX);
                slides.setTargetLength(moveToY);
                arm.setArmAngle(moveToArmAngle);

                if(arm.checkReady() && slides.inPosition(0.5)){
                    state = State.BUFFER;
                    currX = moveToX;
                    currY = moveToY;
                    currArmAngle = moveToArmAngle;
                }
                break;
            case RELEASE:
                arm.openClaw();
                arm.setDiffy(0.0, 0.0);
                if(arm.checkReady()){
                    state = State.BUFFER;
                }
                break;
            case BUFFER:
                //facing back
                if(currArmAngle > Math.toRadians(90.0)){
                    setDepositPositions(currX + 1.0, currY - 1.0);
                }else{
                    setDepositPositions(currX - 1.0, currY - 1.0);
                }
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
            case RETRACT:
                //TODO: May need a separate reset function as setDepositPositions would put claw in this spot
                setDepositPositions(0.0, arm.getArmLength());
                calculateMoveTo();

                arm.setMgnPosition(moveToX);
                slides.setTargetLength(moveToY);
                arm.setArmAngle(moveToArmAngle);
                arm.openClaw();

                if(arm.checkReady() && slides.inPosition(0.5)){
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

    public boolean isSampleDepositDone() {
        return state == State.IDLE;
    }

    public void startOuttake() {
        // TODO Fill in method stub
    }

    public boolean isOuttakeDone() {
        // TODO Fill in method stub
        return true;
    }

    public void grabSpecimen() {
        state = State.GRAB_SET;
    }

    public boolean isSpecimenReady() {
        return state == State.READY;
    }

    public void startSpecimenDeposit() {
        state = State.SPECIMEN_RAISE;
    }

    public boolean isSpecimenDepositDone() {
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
