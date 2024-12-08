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

    private final double intakeX = 10.0, intakeY = -2.0, intakePrepareY = -1.0;
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

        switch(state){
            case IDLE:
                break;
            case TRANSFER_PREPARE_1:
                setDepositPositions(intakeX, intakePrepareY);
                calculateMoveTo();

                updatePositions();

                if(arm.inPosition() && slides.inPosition(0.5)){
                    state = State.TRANSFER_PREPARE_2;
                }
                break;
            case TRANSFER_PREPARE_2:
                arm.setClawRotation(arm.getArmRotation() - Math.PI/2, 1.0);

                if(arm.inPosition()){
                    state = State.TRANSFER_WAIT;
                }
                break;
        }
    }

    public void updatePositions(){
        arm.setHorizontalPos(moveToX, 1.0);
        slides.setTargetLength(moveToY);
        arm.setArmRotation(moveToArmAngle, 1.0);
    }

    /*
    public void prepareTransfer() {

    }

    public void startTransfer() {

    }

    public boolean isSampleReady() {

    }

    public void startSampleDeposit() {

    }

    public void finishSampleDeposit() {

    }

    public boolean isSampleDepositDone() {

    }

    public void startOuttake() {

    }

    public boolean isOuttakeDone() {

    }

    public void grabSpecimen() {

    }

    public void finishSpecimenGrab() {

    }

    public boolean isSpecimenReady() {

    }

    public void startSpecimenDeposit() {

    }

    public void finishSpecimenDeposit() {

    }

    public boolean isSpecimenDepositDone() {

    }

    public void retract() {

    }

    public boolean isRetractDone(){

    }
    */

    public void setDepositPositions(double x, double y){
        targetX = x;
        targetY = y;
    }

    public void calculateMoveTo(){
        double baseX = arm.getHorizontalPos();
        double baseY = slides.getLength();

        //check if only horizontal movement is possible now to minimize slides usage
        if(Math.abs(baseY - targetY) < arm.armLength && targetX >= 0.0){
            double pos1 = targetX - Math.sqrt(arm.armLength * arm.armLength - (baseY - targetY) * (baseY - targetY));
            double pos2 = targetX + Math.sqrt(arm.armLength * arm.armLength - (baseY - targetY) * (baseY - targetY));
            if(Math.abs(pos1 - baseX) < Math.abs(pos2 - baseX) && pos1 >= 0.0){
                moveToX = pos1;
            }else{
                moveToX = pos2;
            }
            moveToY = baseY;
            moveToArmAngle = Math.atan2(moveToY - targetY, moveToX - targetX);
        }else{
            double slope = (baseY - targetY)/(baseX - targetX);
            double pos1x = Utils.minMaxClip(targetX - Math.sqrt(arm.armLength * arm.armLength/(slope * slope + 1)), 0.0, 11.816);
            double pos1y = Utils.minMaxClip(slope * (pos1x - targetX) + targetY, 0.0, 50.0);
            double pos2x = Utils.minMaxClip(targetX + Math.sqrt(arm.armLength * arm.armLength/(slope * slope + 1)), 0.0, 11.816);
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
