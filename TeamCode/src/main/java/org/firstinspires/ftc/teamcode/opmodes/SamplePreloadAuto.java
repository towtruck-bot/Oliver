package org.firstinspires.ftc.teamcode.opmodes;

import android.util.Log;

import com.acmerobotics.dashboard.config.Config;
import com.google.ar.core.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.RobotLog;
import com.sun.tools.javac.code.TargetType;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.subsystems.deposit.nDeposit;
import org.firstinspires.ftc.teamcode.subsystems.drive.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.drive.Spline;
import org.firstinspires.ftc.teamcode.subsystems.intake.IntakeTurret;
import org.firstinspires.ftc.teamcode.subsystems.intake.nClawIntake;
import org.firstinspires.ftc.teamcode.utils.Globals;
import org.firstinspires.ftc.teamcode.utils.LogUtil;
import org.firstinspires.ftc.teamcode.utils.Pose2d;
import org.firstinspires.ftc.teamcode.utils.RunMode;
import org.firstinspires.ftc.teamcode.utils.Utils;
import org.firstinspires.ftc.teamcode.vision.LLBlockDetectionPostProcessor;

import java.util.ArrayList;
import java.util.LinkedList;

@Autonomous(name = "Sample Auto", preselectTeleOp = "A. Teleop")
@Config
public class SamplePreloadAuto extends LinearOpMode {
    private Robot robot;

    // Block positions
    public static double bx1 = 50.4, by1 = 28, bx2 = 59.6, by2 = 28, bx3 = 69, by3 = 28;

    // P/G/C depo positions
    public static double dx1 = 63, dy1 = 53.3;
    public static double dx2 = 64.3, dy2 = 53.6;
    public static double dx3 = 65.395 + 1.05 * Math.cos(Math.toRadians(272.0638)), dy3 = 54.38457 + 1.05 * Math.sin(Math.toRadians(272.0638));

    public int targetSampleIndex = 0;
    public long startTime = System.currentTimeMillis();

    public void runOpMode() {
        Globals.isRed = false;
        Globals.RUNMODE = RunMode.AUTO;
        Globals.hasSamplePreload = true;
        Globals.hasSpecimenPreload = false;

        robot = new Robot(hardwareMap);
        robot.setAbortChecker(() -> !isStopRequested());
        LogUtil.init();

        robot.sensors.resetPosAndIMU();

        robot.ndeposit.state = nDeposit.State.HOLD;
        robot.nclawIntake.setGrabMethod(nClawIntake.GrabMethod.MANUAL_TARGET);
        robot.nclawIntake.setTargetType(nClawIntake.Target.GLOBAL);
        robot.nclawIntake.setGrab(false);
        robot.nclawIntake.setRetryGrab(true);

        robot.ndeposit.presetDepositHeight(false, true, false);
        robot.nclawIntake.disableRestrictedHoldPos();
        robot.nclawIntake.setAutoEnableCamera(false);

        SubmersibleSelectionGUI gui = new SubmersibleSelectionGUI();

        while (opModeInInit() && !isStopRequested()) {
            gui.drawSub(gamepad1, telemetry);
        }

        for (int i = 0; i < 5; ++i) {
            robot.sensors.setOdometryPosition(48.0 - Globals.ROBOT_REVERSE_LENGTH, 72.0 - Globals.ROBOT_WIDTH / 2, Math.PI);
            robot.update();
        }
        LogUtil.drivePositionReset = true;
        robot.update();

        // Sorting Driver input in order of closest target
        ArrayList<Pose2d> input = gui.getDriverSelect();
        ArrayList<Pose2d> targets = new ArrayList<>();
        boolean[] grabbed = new boolean[input.size()];
        Pose2d targetPoint = new Pose2d(7, 12);

        int len = input.size();
        for (int i = 0; i < len; i++) {
            Pose2d best = new Pose2d(100, 100);
            for (Pose2d p : input) {
                best = targetPoint.getDistanceFromPoint(p) < targetPoint.getDistanceFromPoint(best) ? p : best;
            }
            targets.add(best);
            input.remove(best);
            grabbed[i] = false;
        }

        Globals.autoStartTime = System.currentTimeMillis();

        // Preload
        robot.ndeposit.startSampleDeposit();
        robot.drivetrain.goToPoint(
                new Pose2d(dx1, dy1, Math.atan2(by1 - dy1, bx1 - dx1)),
                false,
                true,
                1.0
        );
        robot.nclawIntake.setExtendoTargetPos(5);
        robot.nclawIntake.setTargetType(nClawIntake.Target.MANUAL);
        robot.nclawIntake.extend();
        robot.update(); // Sigh.. - Eric

        robot.waitWhile(() -> robot.drivetrain.targetPoint.getDistanceFromPoint(robot.sensors.getOdometryPosition()) > 10.5);
        robot.nclawIntake.setExtendoTargetPos(15);
        robot.update();

        robot.waitWhile(() -> !robot.ndeposit.isDepositReady() || robot.drivetrain.isBusy());
        robot.ndeposit.deposit();
        robot.nclawIntake.setTargetPose(new Pose2d(bx1, by1, Math.PI / 2));
        robot.nclawIntake.setTargetType(nClawIntake.Target.GLOBAL);
        robot.waitWhile(() -> !robot.ndeposit.isDepositFinished());

        // Depo Ground 1 + intake Ground 2
        robot.waitWhile(() -> !robot.nclawIntake.isExtended());
        robot.nclawIntake.setGrab(true);
        robot.waitWhile(() -> !robot.nclawIntake.hasSample());

        // Go to under bucket
        robot.drivetrain.goToPoint(
                new Pose2d(dx2, dy2, Math.toRadians(257)),
                false,
                true,
                1.0
        );
        robot.waitWhile(() -> !robot.nclawIntake.isTransferReady());
        robot.ndeposit.startSampleDeposit(); // This effectively buffers it
        robot.nclawIntake.finishTransfer();
        robot.ndeposit.finishTransfer();

        robot.waitWhile(() -> robot.nclawIntake.hasSample());
        robot.nclawIntake.setGrab(false);
        robot.nclawIntake.setTargetPose(new Pose2d(bx2, by2, Math.PI / 2));
        robot.nclawIntake.extend();

        robot.waitWhile(() -> robot.drivetrain.isBusy() || !robot.ndeposit.isDepositReady());
        robot.ndeposit.deposit();
        robot.waitWhile(() -> !robot.ndeposit.isDepositFinished());

        // Depo Ground 2 + intake Ground 3
        robot.waitWhile(() -> !robot.nclawIntake.isExtended());
        robot.nclawIntake.setGrab(true);
        robot.waitWhile(() -> !robot.nclawIntake.hasSample());
        robot.ndeposit.presetDepositHeight(false, true, true);
        robot.nclawIntake.enableRestrictedHoldPos();
        // Go to under bucket
        robot.drivetrain.goToPoint(
                new Pose2d(dx3, dy3, Math.toRadians(272.0638)),
                true,
                true,
                1.0
        );
        robot.waitWhile(() -> !robot.nclawIntake.isTransferReady() || !robot.ndeposit.isTransferReady());
        robot.ndeposit.startSampleDeposit(); // This effectively buffers it
        robot.nclawIntake.finishTransfer();
        robot.ndeposit.finishTransfer();

        robot.waitWhile(() -> robot.nclawIntake.hasSample());
        robot.nclawIntake.setGrab(false);
        robot.nclawIntake.setTargetPose(new Pose2d(bx3, by3, Math.PI / 2));
        robot.nclawIntake.extend();

        robot.waitWhile(() -> robot.drivetrain.isBusy() || !robot.ndeposit.isDepositReady());
        robot.drivetrain.putDownBreakPad();
        robot.ndeposit.deposit();
        robot.waitWhile(() -> !robot.ndeposit.isDepositFinished());

        // Depo Ground 3
        robot.waitWhile(() -> !robot.nclawIntake.isExtended());
        robot.nclawIntake.setGrab(true);
        robot.waitWhile(() -> !robot.nclawIntake.hasSample());
        // Go to under bucket
        /*robot.drivetrain.goToPoint(
                new Pose2d(dx3, dy3, Math.toRadians(272.0638)),
                true,
                true,
                1.0
        );*/
        robot.waitWhile(() -> !robot.nclawIntake.isTransferReady() || !robot.ndeposit.isTransferReady());
        robot.ndeposit.startSampleDeposit();
        robot.nclawIntake.finishTransfer();
        robot.ndeposit.finishTransfer();
        robot.drivetrain.raiseBreakPad();

        //robot.waitWhile(() -> !robot.ndeposit.isTransferFinished());
        //robot.ndeposit.deposit();

        //robot.waitWhile(() -> !robot.ndeposit.isDepositFinished());


        robot.nclawIntake.setGrabMethod(nClawIntake.GrabMethod.SEARCH_HOVER_MG);
        robot.waitWhile(() -> robot.nclawIntake.hasSample());

        Pose2d pickUpper = new Pose2d(0, 0, 0);

        while (targetSampleIndex < targets.size() && !isStopRequested()) {
            pickUpper = targets.get(targetSampleIndex);
            Log.i("JAMES", pickUpper.x + " " + pickUpper.y + " " + pickUpper.heading);

            robot.waitWhile(() -> robot.drivetrain.isBusy() || !robot.ndeposit.isDepositReady());
            robot.ndeposit.deposit();

            robot.waitWhile(() -> !robot.ndeposit.isDepositFinished());
            robot.ndeposit.presetDepositHeight(false, true, false);
            robot.nclawIntake.disableRestrictedHoldPos();
            robot.nclawIntake.removeKnown();
            robot.nclawIntake.setExtendoTargetPos(10);
            robot.nclawIntake.extend();

            robot.drivetrain.goToPoint(
                    new Pose2d(33.0, Utils.minMaxClip(pickUpper.y, -17, 17), Math.toRadians(210)),
                    false,
                    false,
                    1.0
            );

            robot.waitWhile(() -> robot.drivetrain.targetPoint.getDistanceFromPoint(robot.sensors.getOdometryPosition()) > 3.5);


            //robot.waitWhile(() -> robot.drivetrain.targetPoint.getDistanceFromPoint(robot.sensors.getOdometryPosition()) > 1.5);
            robot.drivetrain.goToPoint(
                    new Pose2d(30.0, Utils.minMaxClip(pickUpper.y, -17, 17), Math.PI),
                    false,
                    true,
                    1.0
            );

            robot.nclawIntake.setKnownIntakePose(new Pose2d(pickUpper.x, pickUpper.y, 0));
            robot.nclawIntake.aimAtKnown();

            robot.waitWhile(() -> !robot.nclawIntake.isExtended());
            robot.nclawIntake.manualEnableCamera();

            startTime = System.currentTimeMillis();
            robot.waitWhile(() -> {
                if(System.currentTimeMillis() - startTime >= 500){
                    grabbed[targetSampleIndex] = true;
                    do {
                        targetSampleIndex = (targetSampleIndex + 1) % targets.size();
                    } while (grabbed[targetSampleIndex]  && !isStopRequested());
                    return false;
                }
                return robot.vision.getClosestValidBlock() == null || !robot.nclawIntake.isExtended();
            });
            robot.nclawIntake.setGrab(true);

            robot.waitWhile(() -> {
                if (robot.nclawIntake.getRetryCounter() >= 2) {
                    robot.nclawIntake.resetRetryCounter();
                    grabbed[targetSampleIndex] = false;

                    do {
                        targetSampleIndex = (targetSampleIndex + 1) % targets.size();
                    } while (grabbed[targetSampleIndex]  && !isStopRequested());

                    Pose2d currentPos = robot.sensors.getOdometryPosition();
                    Pose2d nextBlock = targets.get(targetSampleIndex);

                    robot.nclawIntake.setKnownIntakePose(nextBlock);
                    robot.nclawIntake.aimAtKnown();

                    double dx = nextBlock.x - currentPos.x;
                    double dy = nextBlock.y - currentPos.y;

                    double relYError = -Math.sin(currentPos.heading) * dx + Math.cos(currentPos.heading)*dy;

                    if (Math.abs(relYError) > IntakeTurret.turretLengthLL * 0.35) {
                        robot.drivetrain.goToPoint(
                                new Pose2d(currentPos.x, currentPos.y, Math.atan2(-nextBlock.y + currentPos.y,-nextBlock.x + currentPos.x)),
                                false,
                                true,
                                0.3
                        );
                    }
                }
                return !robot.nclawIntake.hasSample();
            });
            grabbed[targetSampleIndex] = true;

            // Go to under bucket
            Spline s2 = new Spline(robot.sensors.getOdometryPosition(), 7)
                    .setReversed(true)
                    .addPoint(new Pose2d(38, robot.sensors.getOdometryPosition().y, Math.toRadians(210)))
                    .addPoint(62, 55, Math.toRadians(242));

            robot.drivetrain.setPath(s2);
            robot.drivetrain.state = Drivetrain.State.FOLLOW_SPLINE;
            robot.drivetrain.setMaxPower(0.5);

            robot.waitWhile(() -> !robot.nclawIntake.isTransferReady() || !robot.ndeposit.isTransferReady());
            robot.nclawIntake.finishTransfer();
            robot.ndeposit.finishTransfer();

            robot.waitWhile(() -> robot.sensors.getOdometryPosition().getDistanceFromPoint(s2.getLastPoint()) > 24);
            robot.ndeposit.startSampleDeposit();

            LinkedList<LLBlockDetectionPostProcessor.Block> blocks = robot.vision.getBlocks();
            LLBlockDetectionPostProcessor.filterBlocks(blocks, (LLBlockDetectionPostProcessor.Block b) -> {
                        Pose2d b_global = b.getGlobalPose();
                        return Math.abs(b_global.getX()) <= 20 && Math.abs(b_global.getY()) < 22;
                    }
            );

            // If another valid block exists at that point, do not go to next best since this is still closests and best area. simply update that point, and then i-- so the i++ works out
            if (LLBlockDetectionPostProcessor.getClosestValidBlock(robot.vision.getOffset(), blocks) != null) {
                targets.set(targetSampleIndex, robot.vision.getClosestValidBlock().getGlobalPose().clone());
            } else {
                targetSampleIndex++;
            }
        }

        /*
        for (int i = 0; i < 3; i++) {
            robot.waitWhile(() -> robot.drivetrain.isBusy() || !robot.ndeposit.isDepositReady());
            robot.ndeposit.deposit();
            robot.waitWhile(() -> !robot.ndeposit.isDepositFinished());
            robot.ndeposit.presetDepositHeight(false, true, false);
            robot.nclawIntake.disableRestrictedHoldPos();
            robot.nclawIntake.removeKnown();
            robot.nclawIntake.setExtendoTargetPos(10);
            robot.nclawIntake.extend();

            robot.drivetrain.goToPoint(
                    new Pose2d(33, pickUp.y, Math.toRadians(210)),
                    false,
                    false,
                    1.0
            );

            robot.waitWhile(() -> robot.drivetrain.targetPoint.getDistanceFromPoint(robot.sensors.getOdometryPosition()) > 3.5);
            robot.drivetrain.goToPoint(
                    new Pose2d(30, pickUp.y, Math.PI),
                    false,
                    true,
                    1.0
            );

            robot.nclawIntake.setKnownIntakePose(new Pose2d(pickUp.x, pickUp.y, 0));

            robot.waitWhile(() -> !robot.nclawIntake.isExtended());
            robot.nclawIntake.manualEnableCamera();

            robot.waitWhile(() -> robot.vision.getClosestValidBlock() == null || !robot.nclawIntake.isExtended());
            robot.nclawIntake.setGrab(true);

            robot.waitWhile(() -> !robot.nclawIntake.hasSample());
            // Go to under bucket
            Spline s2 = new Spline(robot.sensors.getOdometryPosition(), 7)
                    .setReversed(true)
                    .addPoint(new Pose2d(33, pickUp.y - 5, Math.toRadians(210)))
                    .addPoint(62, 55, Math.toRadians(242));
            robot.drivetrain.setPath(s2);
            robot.drivetrain.state = Drivetrain.State.FOLLOW_SPLINE;
            robot.drivetrain.setMaxPower(0.5);
            robot.waitWhile(() -> !robot.nclawIntake.isTransferReady() || !robot.ndeposit.isTransferReady());
            robot.nclawIntake.finishTransfer();
            robot.ndeposit.finishTransfer();

            robot.waitWhile(() -> robot.sensors.getOdometryPosition().getDistanceFromPoint(s2.getLastPoint()) > 24);
            robot.ndeposit.startSampleDeposit();

            LinkedList<LLBlockDetectionPostProcessor.Block> blocks = robot.vision.getBlocks();
            LLBlockDetectionPostProcessor.filterBlocks(blocks, (LLBlockDetectionPostProcessor.Block b) -> {
                        Pose2d b_global = b.getGlobalPose();
                        return Math.abs(b_global.getX()) <= 20 && Math.abs(b_global.getY()) < 22;
                    }
            );

            if (LLBlockDetectionPostProcessor.getClosestValidBlock(robot.vision.getOffset(), blocks) != null)
                pickUp = robot.vision.getClosestValidBlock().getGlobalPose().clone();
            else
                pickUp.y -= 4;
        }
        */

        RobotLog.i("Auto total time: " + (System.currentTimeMillis() - Globals.autoStartTime));
    }

//    public void avoidPeckyThing(){
//        while(!robot.nclawIntake.hasSample()){
//
//        }
//    }
}

/*
        robot.goToPoint(
            new Pose2d(dx3, dy3, Math.atan2(by3 - dy3, bx3 - dx3)),
            () -> !(robot.drivetrain.targetPoint.getDistanceFromPoint(robot.sensors.getOdometryPosition()) < 7),
            false,
            true,
            1.0
        );
        robot.waitWhile(() -> !robot.nclawIntake.isExtended() || robot.drivetrain.isBusy());
        robot.nclawIntake.grab(true);
        robot.waitWhile(() -> !robot.nclawIntake.grabFinished());
        robot.drivetrain.goToPoint(
            new Pose2d(dx3, dy3, Math.toRadians(260)),
            false,
            true,
            1.0
        );
        robot.nclawIntake.retract();
        robot.ndeposit.bufferUpwards();
        robot.ndeposit.prepareTransfer();
        robot.waitWhile(() -> !robot.nclawIntake.isRetracted());

        robot.ndeposit.startTransfer();
        robot.waitWhile(() -> !robot.ndeposit.isSampleReady());

        robot.ndeposit.startSampleDeposit();
        robot.waitWhile(() -> robot.drivetrain.isBusy() || !robot.ndeposit.isSampleUp());
        robot.ndeposit.finishSampleDeposit();
        robot.waitWhile(() -> !robot.ndeposit.isSampleDepositDone());

        for(int cycle = 1; cycle <= 3; cycle++) {
            // Spline to submersible
            robot.drivetrain.setFinalAdjustment(false);
            robot.drivetrain.setPath(
                new Spline(new Pose2d(52.0, 24.0, -Math.PI / 2),4)
                    .addPoint(new Pose2d(35.5, 14.0, Math.PI))
            );
            robot.drivetrain.state = Drivetrain.State.FOLLOW_SPLINE;

            // Pre-extension
            robot.waitWhile(() -> robot.sensors.getOdometryPosition().getDistanceFromPoint(robot.drivetrain.getPath().getLastPoint()) < 15);
            robot.setIntakeExtension(12.0);
            robot.nclawIntake.extend();
            // robot.waitWhile(() -> robot.drivetrain.isBusy());

            // phony vision
            robot.drivetrain.goToPoint(new Pose2d(0, 0), true, true, true, 1.0, false);
            robot.nclawIntake.extend();
            while (!robot.nclawIntake.isExtensionAtTarget()) {
                robot.nclawIntake.setIntakeTargetPos(robot.drivetrain.calcExtension(robot.sensors.getOdometryPosition(), new Pose2d(0, 0)));
                robot.update();
            }

            robot.drivetrain.targetPoint = robot.sensors.getOdometryPosition();
            robot.drivetrain.state = Drivetrain.State.WAIT_AT_POINT;

            // Grab at intended position
            robot.grab(true);
            robot.nclawIntake.grab(true);
            robot.waitWhile(() -> !robot.nclawIntake.grabFinished());

            // Go to bottom of high bucket to ndeposit
            robot.drivetrain.goToPoint(
                    new Pose2d(dx4, dy4, Math.atan2(dy4 - 72.0, dx4 - 72.0) + Math.PI),
                    true,
                    true,
                    1.0);
            robot.drivetrain.setFinalAdjustment(false);
            robot.drivetrain.setPath(
                new Spline(new Pose2d(52.0, 24.0, Math.PI),4)
                    .addPoint(new Pose2d(dx3, dy3, Math.toRadians(260)))
            );
            robot.drivetrain.state = Drivetrain.State.FOLLOW_SPLINE;
            robot.nclawIntake.retract();
            robot.ndeposit.bufferUpwards();
            robot.ndeposit.prepareTransfer();
            robot.waitWhile(() -> !robot.nclawIntake.isRetracted());

            // Transfer
            robot.ndeposit.startTransfer();
            robot.waitWhile(() -> !robot.ndeposit.isSampleReady());

            // Raise and ndeposit when at point
            robot.ndeposit.startSampleDeposit();
            robot.waitWhile(() -> robot.drivetrain.isBusy());
            robot.drivetrain.goToPoint(new Pose2d(dx3, dy3, Math.toRadians(260)), false, false, 1.0);
            robot.waitWhile(() -> !robot.ndeposit.isSampleUp());
            robot.ndeposit.finishSampleDeposit();
            robot.waitWhile(() -> !robot.ndeposit.isSampleDepositDone());
        }
    }

        moveToBelowBucket();
        initialMove();
        score();

        if (enableg1) {
            getGround(bx1, by);
            moveToBelowBucket();
            score();
        }

        if (enableg2) {
            getGround(bx2, by);
            moveToBelowBucket();
            score();
        }

        if (enableg3) {
            getGround(bx3, by);
            moveToBelowBucket();
            score();
        }

        if (enabler) {
            goToTeleOpStart();
        } else {
            robot.waitWhile(() -> !robot.ndeposit.isRetractDone());
        }

    public void doInitialization() {
        Globals.isRed = false;
        Globals.RUNMODE = RunMode.AUTO;
        Globals.hasSamplePreload = true;
        Globals.hasSpecimenPreload = false;

        robot = new Robot(hardwareMap);
        robot.setAbortChecker(() -> !isStopRequested());
        LogUtil.init();

        robot.sensors.resetPosAndIMU();

        while (opModeInInit() && !isStopRequested()) {
            //robot.sensors.setOdometryPosition(48.0 - Globals.ROBOT_WIDTH / 2.0, 72.0 - Globals.ROBOT_FORWARD_LENGTH, Math.PI/2);
            robot.sensors.setOdometryPosition(48.0 - Globals.ROBOT_FORWARD_LENGTH / 2, 72.0 - Globals.ROBOT_WIDTH / 2, Math.PI);
            robot.updateDepositHeights(false, true);

            robot.update();
        }
    }

    public void scorePreload(){
        robot.ndeposit.startSampleDeposit();

        robot.nclawIntake.setIntakeTargetPos(robot.drivetrain.calcExtension(new Pose2d(65, 54), new Pose2d(bx1, by)));

        double old = OldDrivetrain.finalTurnThreshold;
        //OldDrivetrain.finalTurnThreshold = 3;
        robot.goToPoint(
            new Pose2d(65, 54, Math.atan2(by - 54, bx1 - 65)), () ->
            !(robot.drivetrain.targetPoint.getDistanceFromPoint(robot.sensors.getOdometryPosition()) < 7),
            false,
            true,
            1.0
        );
        robot.nclawIntake.extend();
        robot.waitWhile(() -> robot.drivetrain.isBusy() || !robot.ndeposit.isSampleUp());
        //OldDrivetrain.finalTurnThreshold = old;

        robot.ndeposit.finishSampleDeposit();
        robot.waitWhile(() -> !robot.ndeposit.isSampleDepositDone());
    }

    public void groundOne(){
        //robot.goToPointWithIntake(new Pose2d(bx1, by), null, true, false, false, 1.0, true);
        robot.waitWhile(() -> !robot.nclawIntake.isExtended());
        robot.nclawIntake.grab(true);
        robot.waitWhile(() -> !robot.nclawIntake.grabFinished());
        robot.nclawIntake.retract();
        robot.waitWhile(() -> !robot.nclawIntake.isRetracted());

        robot.ndeposit.startTransfer();
    }

    public void initialMove(){
        robot.updateDepositHeights(false, true);
        robot.setNextState(Robot.NextState.DEPOSIT);

        robot.goToPoint(new Pose2d(62, 56, Math.PI * 17/12), null, false, false, 0.8);
    }

    public void moveToBelowBucket() {
        // robot current state, SAMPLE_READY
        robot.goToPoint(new Pose2d(sx, sy, sh), null, false, true, 0.9);
        robot.waitWhile(() -> !robot.nclawIntake.isRetracted());

        // raise slides
        robot.updateDepositHeights(false, true);
        robot.setNextState(Robot.NextState.DEPOSIT);

        // wait for full raise
        robot.waitWhile(() -> !robot.ndeposit.isSampleUp());
    }

    public void score() {
        // move in (robot current state, DEPOSIT_BUCKET)
        robot.goToPoint(new Pose2d(sx, sy, sh), null, false, true, 0.9);

        // release sample
        robot.setNextState(Robot.NextState.DONE);
        //robot.waitFor(215);

        // wait for full retract
        robot.waitWhile(() ->  !robot.ndeposit.safeToMove());
    }

    public void getGround(double bx, double by) {
        robot.goToPointWithIntake(new Pose2d(bx, by, Math.toRadians(-90)), null, true, false, true, 0.9, true);

    }

    public void goToTeleOpStart() {
        // prepare for teleop
        robot.goToPoint(new Pose2d(fx1, fy1, fh1), null, false, false, 0.9);
        robot.ndeposit.holdSlides = true;
        robot.ndeposit.setDepositHeight(9.6);
        robot.goToPoint(new Pose2d(fx, fy, fh), null, true, true, 0.9);
        robot.waitWhile(() -> true);
    }
*/