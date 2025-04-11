package org.firstinspires.ftc.teamcode.subsystems.deposit;

import android.util.Log;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.utils.LogUtil;
import org.firstinspires.ftc.teamcode.utils.Utils;
import org.firstinspires.ftc.teamcode.utils.TelemetryUtil;

@Config
public class nDeposit {
    public enum State {
        IDLE,
        TRANSFER_BUFFER,
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
    };
    public HangState hangState = HangState.OFF;
    public boolean holdSlides = false;

    private final Robot robot;
    public final Slides slides;
    private final Arm arm;

    public static double transferArm = 0.4578, transferClaw = -0.7434, transferBufferZ = 10.9, transferZ = 7.5;
    public static double holdArm = -0.4646, holdClaw = -0.6533, holdZ = 0.0;
    public static double raiseArmBufferRotation = 0.346,  sampleArm = -2.177, sampleClaw = 0.4506;
    public static double sampleLZ = 20, sampleHZ = 29.2, speciZ = 18, targetZ = sampleHZ;
    public static double outtakeArm = -2.9225, outtakeClaw = -0.00169, outtakeZ = 0.0;
    public static double specimenIntakeArm = -2.9278, specimenIntakeClaw = 0.3436, specimenIntakeZ = 0;
    public static double specimenDepositArm = -0.1, specimenDepositClaw = -1.2;
    public static double minVel = 1.25;
    private boolean holding = false;
    private long depositStart = 0;
    private double slidesVelWeightedAvg = 0;

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
        slides.update();
        TelemetryUtil.packet.put("Deposit : state", state);
        TelemetryUtil.packet.put("depositState", state);
        LogUtil.depositState.set(this.state.toString());

        switch (state) {
            case IDLE:
                moveToHoldPoses();

                if (transferRequested) {
                    state = State.TRANSFER_BUFFER;
                    transferRequested = false;
                }
                if (specimenIntakeRequested) {
                    state = State.SPECIMEN_INTAKE_WAIT;
                    specimenIntakeRequested = false;
                }
                break;

            case TRANSFER_BUFFER:
                slides.setTargetLength(transferBufferZ);
                arm.setArmRotation(transferArm, 1.0);
                arm.setClawRotation(transferClaw, 1.0);

                arm.clawOpen();

                if (requestFinishTransfer && robot.nclawIntake.isTransferReady() && slides.inPosition(1) && arm.inPosition()) {
                    state = State.TRANSFER_WAIT;
                    slidesVelWeightedAvg = slides.getVel();
                }

                break;

            case TRANSFER_WAIT:
                slides.setTargetLength(transferZ);
                arm.setArmRotation(transferArm, 1.0);
                arm.setClawRotation(transferClaw, 1.0);

                arm.clawOpen();
                slidesVelWeightedAvg = slides.getVel() * 0.5 + slidesVelWeightedAvg * 0.5;

                if (slides.getLength() <= transferZ + 0.2 && Math.abs(slidesVelWeightedAvg) < minVel) {
                    state = State.TRANSFER_FINISH;
                    //robot.nclawIntake.finishTransfer(); // Just to make sure you're not being stupid - Eric
                    requestFinishTransfer = false;
                }

                break;
            case TRANSFER_FINISH:
                // Grab the thing
                slides.setTargetLength(transferZ);
                arm.setArmRotation(transferArm, 1.0);
                arm.setClawRotation(transferClaw, 1.0);

                arm.clawClose();

                if (arm.clawInPosition()) {
                    holding = true;
                    state = State.HOLD;
                }

                // Considering the speed of the slides and how stupid it would look to jitter this, we should probably perform hold checks here
                // BUT. It's not an immediate issue so I don't care. With love, -Eric

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
                slides.setTargetLength(targetZ);
                arm.setArmRotation(raiseArmBufferRotation, 1.0);
                arm.setClawRotation(sampleClaw, 0.3);

                arm.clawClose();

                if (slides.getLength() >= targetZ - 7) {
                    state = State.SAMPLE_WAIT;
                }
                break;
            case SAMPLE_WAIT:
                slides.setTargetLength(targetZ);
                arm.setArmRotation(sampleArm, 1.0);
                arm.setClawRotation(sampleClaw, 1.0);

                arm.clawClose();

                if (arm.inPosition() && slides.inPosition(1.0) && releaseRequested) {
                    state = State.SAMPLE_DEPOSIT;
                    releaseRequested = false;
                    depositStart = System.currentTimeMillis();
                }
                break;
            case SAMPLE_DEPOSIT:
                slides.setTargetLength(targetZ);
                arm.setArmRotation(sampleArm, 1.0);
                arm.setClawRotation(sampleClaw, 1.0);

                if (System.currentTimeMillis() - depositStart > 75)
                    arm.clawOpen();

                if (arm.clawInPosition() && System.currentTimeMillis() - depositStart > 125) {
                    state = State.RETRACT;
                    holding = false;
                }
                break;
            case OUTTAKE:
                // I don't know what the hell outtake does and why it like instantly pops the thing off but I trust its useful - Eric
                slides.setTargetLength(outtakeZ);
                arm.setArmRotation(outtakeArm, 1.0);
                arm.setClawRotation(outtakeClaw, 1.0);

                if (arm.inPosition()) {
                    holding = false;
                    state = State.IDLE;
                    arm.clawOpen();
                }
                break;
            case SPECIMEN_INTAKE_WAIT:
                slides.setTargetLength(specimenIntakeZ);
                arm.setArmRotation(specimenIntakeArm, 1.0);
                arm.setClawRotation(specimenIntakeClaw, 1.0);

                arm.clawOpen();

                if (slides.inPosition(1.0) && arm.inPosition() && grabRequested) {
                    state = State.SPECIMEN_GRAB;
                    grabRequested = false;
                }
                break;
            case SPECIMEN_GRAB:
                slides.setTargetLength(specimenIntakeZ);
                arm.setArmRotation(specimenIntakeArm, 1.0);
                arm.setClawRotation(specimenIntakeClaw, 1.0);

                arm.clawClose();

                if (arm.clawInPosition()) {
                    state = State.HOLD;
                }
                break;
            case SPECIMEN_RAISE:
                slides.setTargetLength(targetZ);
                arm.setArmRotation(specimenDepositArm, 1.0);
                arm.setClawRotation(specimenDepositClaw, 1.0);

                if (arm.inPosition() && slides.inPosition(0.5) && releaseRequested) {
                    state = State.SPECIMEN_DEPOSIT;
                    releaseRequested = false;
                }
                break;
            case SPECIMEN_DEPOSIT:
                slides.setTargetLength(targetZ);
                arm.setArmRotation(specimenDepositArm, 1.0);
                arm.setClawRotation(specimenDepositClaw, 1.0);

                arm.clawOpen();

                if (arm.clawInPosition()) {
                    state = State.RETRACT;
                    holding = false;
                }
                break;
            case RETRACT:
                moveToHoldPoses();

                if (arm.inPosition() && slides.inPosition(0.5)) {
                    state = State.IDLE;
                }

                if (transferRequested) {
                    state = State.TRANSFER_BUFFER;
                    transferRequested = false;
                }
                break;
            case TEST:
                break;
        }

        if (hangState == HangState.PULL) {
            slides.setTargetPowerFORCED(-0.9);
            targetZ = slides.getLength() - 0.5;
        } else if (hangState == HangState.OUT) {
            slides.setTargetPowerFORCED(0.7);
            targetZ = slides.getLength() + 0.5;
        }
        if (hangState != HangState.OFF) holdSlides = true;
        if (holdSlides) slides.setTargetLength(targetZ);
        hangState = HangState.OFF;

        TelemetryUtil.packet.put("arm target", arm.armRotation.getTargetAngle());
        TelemetryUtil.packet.put("arm inPosition", arm.armInPosition());
        TelemetryUtil.packet.put("claw target", arm.clawRotation.getTargetAngle());
        TelemetryUtil.packet.put("claw inPosition", arm.clawInPosition());
        TelemetryUtil.packet.put("slides target", slides.getTargetLength());
        TelemetryUtil.packet.put("slides inPosition", slides.inPosition(1.0));
    }

    public void setByCoords(double armRad, double clawRad, double slidesHeight){
        arm.setArmRotation(armRad, 1.0);
        arm.setClawRotation(clawRad, 1.0);
        slides.setTargetLength(slidesHeight);
    }

    public void setDepositHeight(double l) {
        targetZ = Utils.minMaxClip(l, 0.0, Slides.maxSlidesHeight);
    }
    public double getDepositHeight() { return targetZ; }
    public void presetDepositHeight(boolean speciMode, boolean high) {
        targetZ = speciMode ? speciZ : high ? sampleHZ : sampleLZ;
    }

    private void moveToHoldPoses() {
        slides.setTargetLength(holdZ);
        arm.setArmRotation(holdArm, 1.0);
        arm.setClawRotation(holdClaw, 1.0);
    }

    public void startTransfer() {
        Log.i("FSM", "nDeposit.startTransfer");
        transferRequested = true;
    }

    public void finishTransfer() {
        Log.i("FSM", "nDeposit.finishTransfer");
        requestFinishTransfer = true;
    }

    public void returnToIdle() { state = State.IDLE; }

    public boolean retractReady() {
        return arm.armRotation.getCurrentAngle() < -0.4 || slides.getLength() >= 3;
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
        Log.i("FSM", "nDeposit.startSampleDeposit");
        sampleDepositRequested = true;
    }

    public void startSpecimenDeposit() {
        specimenDepositRequested = true;
    }

    public void startSpecimenIntake() {
        specimenIntakeRequested = true;
    }

    public void deposit() {
        Log.i("FSM", "nDeposit.deposit");
        if (state == State.SAMPLE_WAIT || state == State.SPECIMEN_RAISE) {
            releaseRequested = true;
        }
    }

    public boolean isDepositingSample() {
        return state == State.SAMPLE_RAISE || state == State.SAMPLE_WAIT || state == State.SAMPLE_DEPOSIT;
    }

    public boolean isTransferReady() {
        return state == State.TRANSFER_BUFFER && arm.inPosition() && slides.inPosition(0.2);
    }

    public boolean isDepositReady() {
        return (state == State.SAMPLE_WAIT ||
                state == State.SPECIMEN_RAISE) &&
                slides.inPosition(1.0) &&
                arm.inPosition();
    }

    public boolean isDepositFinished() {
        return ((state == State.SAMPLE_DEPOSIT ||
                state == State.SPECIMEN_DEPOSIT) &&
                arm.clawInPosition()) ||
                state == State.RETRACT ||
                state == State.IDLE;
    }

    public boolean isHolding() {
        return holding;
    }
}
