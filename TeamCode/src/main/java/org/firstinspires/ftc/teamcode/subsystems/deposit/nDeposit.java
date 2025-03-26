package org.firstinspires.ftc.teamcode.subsystems.deposit;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.utils.Utils;

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
        GRAB,
        GRAB_FINISH,
        SPECIMEN_RAISE,
        SPECIMEN_DEPOSIT,
        RETRACT
    };
    nDepositState ndepositState = nDepositState.IDLE;

    public enum HangState{
        OUT,
        PULL,
        OFF
    };
    public HangState hangState = HangState.OFF;
    public boolean holdSlides = false;

    private Robot robot;
    private Slides slides;
    private Arm arm;

    // TODO: All values below (v) are estimates and may not scale correctly with the 0 positions. Make sure to tune!
    public static double transferArm = Math.toRadians(-45), transferClaw = 0.0, transferPrepY = 7.0, transferY = 5.0;
    public static double holdArm = Math.toRadians(90), holdClaw = 0.0, holdY = 0.0;
    public static double sampleArm = Math.toRadians(150), sampleClaw = Math.toRadians(30), sampleY = 32.5, sampleRetract = Math.toRadians(90);
    public static double outtakeArm = Math.toRadians(135), outtakeClaw = Math.toRadians(45), outtakeY = 0.0;
    public static double grabArm = Math.toRadians(180.0), grabClaw = 0.0, grabY = 0.0;
    public static double speciArm = Math.toRadians(30.0), speciClaw = Math.toRadians(-50.0), speciY = 18.6;

    private boolean prepare = false, transfer = false, deposit = false, sample = false, outtake = false, grab = false, speci = false;

    private double targetY = 0.0;

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

                if(prepare){
                    ndepositState = nDepositState.TRANSFER_PREP;
                    prepare = false;
                }
                break;
            case TRANSFER_PREP:
                slides.setTargetLength(targetY);
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
                slides.setTargetLength(targetY);
                arm.setArmRotation(transferArm, 1.0);
                arm.setClawRotation(transferClaw, 1.0);

                arm.sampleOpen();

                if(arm.inPosition() && slides.inPosition(0.5)){
                    ndepositState = nDepositState.TRANSFER_FINISH;
                }
                break;
            case TRANSFER_FINISH:
                slides.setTargetLength(targetY);
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

                if(sample){
                    ndepositState = nDepositState.SAMPLE_RAISE;
                    sample = false;
                }

                if(outtake){
                    ndepositState = nDepositState.OUTTAKE;
                    outtake = false;
                }

                if(speci){
                    ndepositState = nDepositState.SPECIMEN_RAISE;
                    speci = false;
                }
                break;
            case SAMPLE_RAISE:
                slides.setTargetLength(targetY);
                arm.setArmRotation(sampleArm, 1.0);
                arm.setClawRotation(sampleClaw, 1.0);

                arm.clawClose();

                if(arm.inPosition() && slides.inPosition(1.0) && deposit){
                    ndepositState = nDepositState.SAMPLE_DEPOSIT;
                    deposit = false;
                }
                break;
            case SAMPLE_DEPOSIT:
                slides.setTargetLength(targetY);
                arm.setArmRotation(sampleArm, 1.0);
                arm.setClawRotation(sampleClaw, 1.0);

                arm.sampleOpen();

                if(arm.clawInPosition()){
                    ndepositState = nDepositState.SAMPLE_RETRACT;
                }
                break;
            case SAMPLE_RETRACT:
                slides.setTargetLength(targetY);
                arm.setArmRotation(sampleRetract, 1.0);
                arm.setClawRotation(holdClaw, 1.0);

                if(arm.armInPosition()){
                    ndepositState = nDepositState.RETRACT;
                }
                break;
            case OUTTAKE:
                slides.setTargetLength(targetY);
                arm.setArmRotation(outtakeArm, 1.0);
                arm.setClawRotation(outtakeClaw, 1.0);

                if(arm.inPosition()){
                    ndepositState = nDepositState.IDLE;
                    arm.sampleOpen();
                }
                break;
            case GRAB:
                slides.setTargetLength(targetY);
                arm.setArmRotation(grabArm, 1.0);
                arm.setClawRotation(grabClaw, 1.0);

                arm.sampleOpen();

                if(grab){
                    ndepositState = nDepositState.GRAB_FINISH;
                    grab = false;
                }
                break;
            case GRAB_FINISH:
                slides.setTargetLength(targetY);
                arm.setArmRotation(grabArm, 1.0);
                arm.setClawRotation(grabClaw, 1.0);

                arm.clawClose();

                if(arm.clawInPosition()){
                    ndepositState = nDepositState.HOLD;
                }
                break;
            case SPECIMEN_RAISE:
                slides.setTargetLength(targetY);
                arm.setArmRotation(speciArm, 1.0);
                arm.setClawRotation(speciClaw, 1.0);

                if(arm.inPosition() && slides.inPosition(0.5) && deposit){
                    ndepositState = nDepositState.SPECIMEN_DEPOSIT;
                    deposit = false;
                }
                break;
            case SPECIMEN_DEPOSIT:
                slides.setTargetLength(targetY);
                arm.setArmRotation(speciArm, 1.0);
                arm.setClawRotation(speciClaw, 1.0);

                arm.sampleOpen();

                if(arm.clawInPosition()){
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

        if(holdSlides){
            slides.setTargetLength(targetY);
        }

        if(hangState == HangState.PULL){
            slides.setTargetPowerFORCED(-0.9);
            targetY = slides.getLength() - 0.5;
        }else if(hangState == HangState.OUT){
            slides.setTargetPowerFORCED(0.7);
            targetY = slides.getLength() + 0.5;
        }

        if(hangState != HangState.OFF){
            holdSlides = true;
        }

        hangState = HangState.OFF;
    }

    public void setDepositHeight(double l){
        targetY = Utils.minMaxClip(l, 0.0, Slides.maxSlidesHeight);
    }

    public void movePoses(){
        slides.setTargetLength(holdY);
        arm.setArmRotation(holdArm, 1.0);
        arm.setClawRotation(holdClaw, 1.0);
    }

    public void prepareTransfer(){
        if(ndepositState == nDepositState.IDLE){
            prepare = true;
            targetY = transferPrepY;
        }
    }

    public void startTransfer(){
        if(ndepositState == nDepositState.TRANSFER_WAIT){
            transfer = true;
            targetY = transferY;
        }
    }

    public boolean isTransferFinished(){
        return ndepositState == nDepositState.HOLD;
    }

    public void outtake(){
        if(ndepositState == nDepositState.HOLD){
            outtake = true;
            targetY = outtakeY;
        }
    }

    public boolean isOuttakeDone(){
        return ndepositState == nDepositState.HOLD;
    }

    public void grab(){
        if(ndepositState == nDepositState.GRAB){
            grab = true;
            targetY = grabY;
        }
    }

    public boolean isGrabDone(){
        return ndepositState == nDepositState.HOLD;
    }

    public void sampleDeposit(){
        if(ndepositState == nDepositState.HOLD){
            sample = true;
            targetY = sampleY;
        }
    }

    public void speciDeposit(){
        if(ndepositState == nDepositState.HOLD){
            speci = true;
            targetY = speciY;
        }
    }

    public void deposit(){
        if(ndepositState == nDepositState.SAMPLE_RAISE || ndepositState == nDepositState.SPECIMEN_RAISE){
            deposit = true;
        }
    }

    public boolean isDepositFinished(){
        return ndepositState == nDepositState.IDLE;
    }
}
