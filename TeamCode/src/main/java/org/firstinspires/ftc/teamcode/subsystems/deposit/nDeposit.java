package org.firstinspires.ftc.teamcode.subsystems.deposit;

import org.firstinspires.ftc.teamcode.Robot;

public class nDeposit {
    public enum nDepositState{
        IDLE,
        TRANSFER_PREP,
        TRANSFER_WAIT,
        TRANSFER_GRAB,
        TRANSFER_FINISH,
        HOLD,
        SAMPLE_RAISE,
        SAMPLE_DEPOSIT,
        SAMPLE_RETRACT,
        OUTTAKE,
        RETRACT
    };
    nDepositState ndepositState = nDepositState.IDLE;

    private Robot robot;
    private Slides slides;
    private Arm arm;

    // TODO: All values below (v) are estimates and may not scale correctly with the 0 positions. Make sure to tune!
    public static double transferArm = Math.toRadians(-45), transferClaw = 0.0, transferPrepY = 7.0, transferY = 5.0;
    public static double holdArm = Math.toRadians(90), holdClaw = 0.0, holdY = 0.0;
    public static double sampleArm = Math.toRadians(150), sampleClaw = Math.toRadians(30), sampleY = 32.5, sampleRetract = Math.toRadians(90);

    private boolean transfer = false, deposit = false;

    public nDeposit(Robot robot){
        this.robot = robot;

        slides = new Slides(this.robot);
        arm = new Arm(this.robot);
    }

    public void update(){
        slides.update();

        switch(ndepositState){
            case IDLE:
                movePoses();
                break;
            case TRANSFER_PREP:
                slides.setTargetLength(transferPrepY);
                arm.setArmRotation(transferArm, 1.0);
                arm.setClawRotation(transferClaw, 1.0);

                arm.sampleOpen();

                if(arm.inPosition() && slides.inPosition(1.0)){
                    ndepositState = nDepositState.TRANSFER_WAIT;
                }
                break;
            case TRANSFER_WAIT:
                if(transfer){
                    ndepositState = nDepositState.TRANSFER_GRAB;
                    transfer = false;
                }
                break;
            case TRANSFER_GRAB:
                slides.setTargetLength(transferY);
                arm.setArmRotation(transferArm, 1.0);
                arm.setClawRotation(transferClaw, 1.0);

                arm.sampleOpen();

                if(arm.inPosition() && slides.inPosition(0.5)){
                    ndepositState = nDepositState.TRANSFER_FINISH;
                }
                break;
            case TRANSFER_FINISH:
                slides.setTargetLength(transferY);
                arm.setArmRotation(transferArm, 1.0);
                arm.setClawRotation(transferClaw, 1.0);

                arm.clawClose();

                if(arm.clawInPosition()){
                    ndepositState = nDepositState.HOLD;
                    robot.nclawIntake.completeIntakeTransfer();
                }
                break;
            case HOLD:
                movePoses();
                break;
            case SAMPLE_RAISE:
                slides.setTargetLength(sampleY);
                arm.setArmRotation(sampleArm, 1.0);
                arm.setClawRotation(sampleClaw, 1.0);

                arm.clawClose();

                if(arm.inPosition() && slides.inPosition(1.0) && deposit){
                    ndepositState = nDepositState.SAMPLE_DEPOSIT;
                    deposit = false;
                }
                break;
            case SAMPLE_DEPOSIT:
                slides.setTargetLength(sampleY);
                arm.setArmRotation(sampleArm, 1.0);
                arm.setClawRotation(sampleClaw, 1.0);

                arm.sampleOpen();

                if(arm.clawInPosition()){
                    ndepositState = nDepositState.SAMPLE_RETRACT;
                }
                break;
            case SAMPLE_RETRACT:
                slides.setTargetLength(sampleY);
                arm.setArmRotation(sampleRetract, 1.0);
                arm.setClawRotation(holdClaw, 1.0);

                if(arm.armInPosition()){
                    ndepositState = nDepositState.RETRACT;
                }
                break;

            case RETRACT:
                movePoses();

                if(arm.inPosition() && slides.inPosition(0.5)){
                    ndepositState = nDepositState.IDLE;
                }
                break;
        }
    }

    public void movePoses(){
        slides.setTargetLength(holdY);
        arm.setArmRotation(holdArm, 1.0);
        arm.setClawRotation(holdClaw, 1.0);
    }

    public void startTransfer(){
        if(ndepositState == nDepositState.TRANSFER_WAIT){
            transfer = true;
        }
    }

    public boolean isTransferFinished(){
        return ndepositState == nDepositState.HOLD;
    }

    public void deposit(){

    }
}
