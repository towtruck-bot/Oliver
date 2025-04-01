package org.firstinspires.ftc.teamcode.subsystems.deposit;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.utils.Utils;
import org.firstinspires.ftc.teamcode.utils.TelemetryUtil;

@Config
public class nDeposit {
    public enum State {
        IDLE,
        TRANSFER_WAIT,
        TRANSFER_FINISH,
        HOLD,
        SAMPLE_RAISE,
        SAMPLE_WAIT,
        SAMPLE_DEPOSIT,
        OUTTAKE,
        SPECIMEN_INTAKE_WAIT,
        SPECIMEN_GRAB,
        SPECIMEN_RAISE,
        SPECIMEN_DEPOSIT,
        RETRACT,
        TEST
    };
    public State state = State.IDLE;

    public enum HangState {
        OUT,
        PULL,
        OFF
    }

    ;
    public HangState hangState = HangState.OFF;

    private Robot robot;
    private Slides slides;
    private Arm arm;

    // TODO: All values below (v) are estimates and may not scale correctly with the 0 positions. Make sure to tune!
    public static double transferArm = 0.5696, transferClaw = 0, transferY = 6.5;
    public static double holdArm = -0.889, holdClaw = 0.6477, holdY = 0.0;
    public static double raiseArmBufferRotation = -1.8472,  sampleArm = -2.9728, sampleClaw = -0.0563, sampleY = 32.5;
    public static double outtakeArm = -2.9225, outtakeClaw = -0.00169, outtakeY = 0.0;
    public static double specimenIntakeArm = -2.9278, specimenIntakeClaw = 0.3436, specimenIntakeY = 0;
    public static double specimenDepositArm = -0.1, specimenDepositClaw = -1.2, specimenDepositY = 18.6;

    private boolean requestFinishTransfer = false,
                    transferRequested = false,
                    releaseRequested = false,
                    sampleDepositRequested = false,
                    outtakeRequested = false,
                    specimenDepositRequested = false,
                    specimenIntakeRequested = false,
                    grabRequested = false;

    public nDeposit(Robot robot) {
        this.robot = robot;

        slides = new Slides(robot);
        arm = new Arm(robot);
    }

    public void update() {
        TelemetryUtil.packet.put("Deposit.state", state);
        slides.update();

        switch (state) {
            case IDLE:
                moveToHoldPoses();

                if (transferRequested) {
                    state = State.TRANSFER_WAIT;
                    transferRequested = false;
                }
                if (specimenIntakeRequested) {
                    state = State.SPECIMEN_INTAKE_WAIT;
                    specimenIntakeRequested = false;
                }
                break;
            case TRANSFER_WAIT:
                slides.setTargetLength(transferY);
                arm.setArmRotation(transferArm, 1.0);
                arm.setClawRotation(transferClaw, 1.0);

                arm.sampleOpen();

                if (requestFinishTransfer) {
                    state = State.TRANSFER_FINISH;
                    //robot.nclawIntake.finishTransfer(); // Just to make sure you're not being stupid - Eric
                    requestFinishTransfer = false;
                }

                break;
            case TRANSFER_FINISH:
                // Grab the thing
                slides.setTargetLength(transferY);
                arm.setArmRotation(transferArm, 1.0);
                arm.setClawRotation(transferClaw, 1.0);

                arm.clawClose();

                if (arm.clawInPosition()) {
                    state = State.HOLD;
                }
                break;
            case HOLD:
                moveToHoldPoses();

                if (sampleDepositRequested) {
                    state = State.SAMPLE_RAISE;
                    sampleDepositRequested = false;
                }

                if (outtakeRequested) {
                    state = State.OUTTAKE;
                    outtakeRequested = false;
                }

                if (specimenDepositRequested) {
                    state = State.SPECIMEN_RAISE;
                    specimenDepositRequested = false;
                }
                break;
            case SAMPLE_RAISE:
                slides.setTargetLength(sampleY);
                arm.setArmRotation(raiseArmBufferRotation, 1.0);
                arm.setClawRotation(sampleClaw, 1.0);

                arm.clawClose();

                if (arm.inPosition()) {
                    state = State.SAMPLE_WAIT;
                }
                break;
            case SAMPLE_WAIT:
                slides.setTargetLength(sampleY);
                arm.setArmRotation(sampleArm, 1.0);
                arm.setClawRotation(sampleClaw, 1.0);

                arm.clawClose();

                if (arm.inPosition() && slides.inPosition(1.0) && releaseRequested) {
                    state = State.SAMPLE_DEPOSIT;
                    releaseRequested = false;
                }
                break;
            case SAMPLE_DEPOSIT:
                slides.setTargetLength(sampleY);
                arm.setArmRotation(sampleArm, 1.0);
                arm.setClawRotation(sampleClaw, 1.0);

                arm.sampleOpen();

                if (arm.clawInPosition()) {
                    state = State.RETRACT;
                }
                break;
            case OUTTAKE:
                // I don't know what the hell outtake does and why it like instantly pops the thing off but I trust its useful - Eric
                slides.setTargetLength(outtakeY);
                arm.setArmRotation(outtakeArm, 1.0);
                arm.setClawRotation(outtakeClaw, 1.0);

                if (arm.inPosition()) {
                    state = State.IDLE;
                    arm.sampleOpen();
                }
                break;
            case SPECIMEN_INTAKE_WAIT:
                slides.setTargetLength(specimenIntakeY);
                arm.setArmRotation(specimenIntakeArm, 1.0);
                arm.setClawRotation(specimenIntakeClaw, 1.0);

                arm.sampleOpen();

                if (slides.inPosition(1.0) && arm.inPosition() && grabRequested) {
                    state = State.SPECIMEN_GRAB;
                }
                break;
            case SPECIMEN_GRAB:
                slides.setTargetLength(specimenIntakeY);
                arm.setArmRotation(specimenIntakeArm, 1.0);
                arm.setClawRotation(specimenIntakeClaw, 1.0);

                arm.clawClose();

                if (arm.clawInPosition()) {
                    state = State.HOLD;
                }
                break;
            case SPECIMEN_RAISE:
                slides.setTargetLength(specimenDepositY);
                arm.setArmRotation(specimenDepositArm, 1.0);
                arm.setClawRotation(specimenDepositClaw, 1.0);

                if (arm.inPosition() && slides.inPosition(0.5) && releaseRequested) {
                    state = State.SPECIMEN_DEPOSIT;
                    releaseRequested = false;
                }
                break;
            case SPECIMEN_DEPOSIT:
                slides.setTargetLength(specimenDepositY);
                arm.setArmRotation(specimenDepositArm, 1.0);
                arm.setClawRotation(specimenDepositClaw, 1.0);

                arm.sampleOpen();

                if (arm.clawInPosition()) {
                    state = State.RETRACT;
                }
                break;
            case RETRACT:
                moveToHoldPoses();

                if (arm.inPosition() && slides.inPosition(0.5)) {
                    state = State.IDLE;
                }
                break;
            case TEST:
                break;
        }

        if (hangState == HangState.PULL) {
            slides.setTargetPowerFORCED(-0.9);
        } else if (hangState == HangState.OUT) {
            slides.setTargetPowerFORCED(0.7);
        }

        TelemetryUtil.packet.put("arm target", arm.armRotation.getTargetAngle());
        TelemetryUtil.packet.put("arm inPosition", arm.armInPosition());
        TelemetryUtil.packet.put("claw target", arm.clawRotation.getTargetAngle());
        TelemetryUtil.packet.put("claw inPosition", arm.clawInPosition());
        TelemetryUtil.packet.put("slides target", slides.getTargetLenght());
        TelemetryUtil.packet.put("slides inPosition", slides.inPosition(1.0));
    }

    public void setByCoords(double armRad, double clawRad, double slidesHeight){
        arm.setArmRotation(armRad, 1.0);
        arm.setClawRotation(clawRad, 1.0);
        slides.setTargetLength(slidesHeight);
    }

    public void setSampleHeight(double l) {
        sampleY = Utils.minMaxClip(l, 0.0, Slides.maxSlidesHeight);
    }

    private void moveToHoldPoses() {
        slides.setTargetLength(holdY);
        arm.setArmRotation(holdArm, 1.0);
        arm.setClawRotation(holdClaw, 1.0);
    }

    public void startTransfer() {
        transferRequested = true;
    }

    public void finishTransfer() {
        requestFinishTransfer = true;
    }

    public boolean retractReady() {
        return arm.armRotation.getCurrentAngle() < -0.4;
    }

    public boolean isTransferFinished() {
        return state == State.HOLD;
    }

    public void outtake() {
        if (state == State.HOLD)
            outtakeRequested = true;
    }

    public boolean isOuttakeDone() {
        return state == State.HOLD;
    }

    public void grab() {
        grabRequested = true;
    }

    public boolean isGrabDone() {
        return state == State.HOLD;
    }

    public void startSampleDeposit() {
        if(state == State.HOLD){
            sampleDepositRequested = true;
        }
    }

    public void startSpecimenDeposit() {
        if (state == State.HOLD)
            specimenDepositRequested = true;
    }

    public void startSpecimenIntake() {
        specimenIntakeRequested = true;
    }

    public void deposit() {
        if (state == State.SAMPLE_WAIT || state == State.SPECIMEN_RAISE) {
            releaseRequested = true;
        }
    }

    public boolean isDepositFinished() {
        return state == State.IDLE;
    }

    public boolean isTransferReady() {
        return state == State.TRANSFER_WAIT && arm.inPosition() && slides.inPosition(0.6);
    }
}
