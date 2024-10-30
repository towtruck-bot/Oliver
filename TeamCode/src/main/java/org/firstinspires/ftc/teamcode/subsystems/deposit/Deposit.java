package org.firstinspires.ftc.teamcode.subsystems.deposit;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.sensors.Sensors;
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
    public Sensors sensors;

    //TODO: Servos have mechanically set 0 positions, make sure nathan xie does it so i can make him give me good zeros
    //TODO: also for max ranges too
    //TODO: make sure axons are in the range i want it to be in
    //TODO: priority servo reverse might not work, very sketchy thing, be careful of that function - profe
    private double currX, currY, currArmAngle;
    private double targetX, targetY;
    private double moveToX, moveToY, moveToArmAngle;
    private boolean tooClose = false, outtaking = false;

    private final double intakeWaitY = -1.0, intakeX = 10.0, intakeY = -2.0; // TODO: Update these values
    private final double sampleBasketX = -2.0, sampleBasketY = 46.0;
    private final double specimenBarX = 10.0, specimenBarY = 27.0;
    private final double outtakeX = -11.816, outtakeY = 0.0, grabX = -5.0, grabY = -4.0;

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
                setDepositPositions(intakeX + Utils.minMaxClip(robot.sensors.getIntakeExtensionPosition(), 0.0, 2.0), intakeWaitY);
                calculateMoveTo();

                //TODO: Added dependency on current arm angle
                //TODO: is the block coming in square face or rectangle face

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
                    state = State.TRANSFER_WAIT;
                    currX = moveToX;
                    currY = moveToY;
                    currArmAngle = moveToArmAngle;
                    arm.closeClaw();
                }
                break;
            case TRANSFER_WAIT:
                arm.setDiffy(Math.toRadians(-90.0) - moveToArmAngle, Math.toRadians(90.0));
                arm.openClaw();
                if(arm.checkReady() && robot.sensors.getIntakeExtensionPosition() == 0.0){
                    state = State.TRANSFER_GRAB;
                }
                break;
            case TRANSFER_GRAB:
                setDepositPositions(intakeX, intakeY);
                calculateMoveTo();

                arm.setMgnPosition(moveToX);
                arm.setArmAngle(moveToArmAngle);
                slides.setTargetLength(moveToY);

                if(arm.checkReady()){
                    arm.closeClaw();
                    if(arm.checkReady() && !outtaking){
                        state = State.READY;
                    }else if(arm.checkReady() && outtaking){
                        state = State.OUTTAKE_MOVE;
                    }
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
            case OUTTAKE_RELEASE:
                arm.openClaw();
                if(arm.checkReady()){
                    state = State.RETRACT;
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
        outtaking = false;
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
        outtaking = true;
        state = State.TRANSFER_START;
    }

    public boolean isOuttakeDone() {
        return state == State.IDLE;
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
