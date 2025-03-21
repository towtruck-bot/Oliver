package org.firstinspires.ftc.teamcode.opmodes;

import android.util.Log;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.subsystems.drive.Spline;
import org.firstinspires.ftc.teamcode.utils.Globals;
import org.firstinspires.ftc.teamcode.utils.LogUtil;
import org.firstinspires.ftc.teamcode.utils.Pose2d;
import org.firstinspires.ftc.teamcode.utils.RunMode;

@Autonomous(name = "SamplePreloadBlueAuto", preselectTeleOp = "A. Teleop")
@Config
public class SamplePreloadBlueAuto extends LinearOpMode {
    private Robot robot;

    public static boolean enableg1 = true, enableg2 = true, enableg3 = true, enabler = true;

    // Block positions
    public static double by1 = 26.3, bx1 = 49.5, bx2 = 59.7, by2 = 27, bx3 = 69.5, by3 = 26.8;

    // GP depo positions
    public static double dx1 = 63.5, dy1 = 54;
    public static double dx2 = 64.5, dy2 = 54;
    public static double dx3 = 63.5, dy3 = 55;
    public static double dx4 = 63.5, dy4 = 54;

    public void runOpMode(){
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
            robot.sensors.setOdometryPosition(48.0 - Globals.ROBOT_REVERSE_LENGTH, 72.0 - Globals.ROBOT_WIDTH / 2, Math.PI);
            robot.updateDepositHeights(false, true);

            robot.update();
        }

        Globals.autoStartTime = System.currentTimeMillis();

        // Preload
        robot.deposit.startSampleDeposit();
        robot.goToPoint(
            new Pose2d(dx1, dy1, Math.atan2(by1 - dy1, bx1 - dx1)),
            () -> !(robot.drivetrain.targetPoint.getDistanceFromPoint(robot.sensors.getOdometryPosition()) < 10),
            false,
            true,
            1.0
        );
        robot.clawIntake.setIntakeTargetPos(robot.drivetrain.calcExtension(new Pose2d(dx1, dy1), new Pose2d(bx1, by1)));
        robot.clawIntake.extend();
        robot.waitWhile(() -> robot.drivetrain.isBusy() || !robot.deposit.isSampleUp());

        robot.deposit.finishSampleDeposit();
        robot.waitWhile(() -> !robot.deposit.isSampleDepositDone());


        // Depo 1
        robot.waitWhile(() -> !robot.clawIntake.isExtended());
        robot.clawIntake.grab(true);
        robot.waitWhile(() -> !robot.clawIntake.grabFinished());
        // Go to under bucket
        robot.drivetrain.goToPoint(
            new Pose2d(dx2, dy2, Math.atan2(by2 - dy2, bx2 - dx2)),
            false,
            true,
            1.0
        );
        robot.clawIntake.retract();
        robot.deposit.bufferUpwards();
        robot.deposit.prepareTransfer();
        robot.waitWhile(() -> !robot.clawIntake.isRetracted());

        robot.deposit.startTransfer();


        robot.waitWhile(() -> !robot.deposit.isSampleReady());
        robot.deposit.startSampleDeposit();
        robot.clawIntake.setIntakeTargetPos(robot.drivetrain.calcExtension(new Pose2d(dx2, dy2), new Pose2d(bx2, by2)));
        robot.clawIntake.extend();
        robot.waitWhile(() -> robot.drivetrain.isBusy() || !robot.deposit.isSampleUp());
        robot.deposit.finishSampleDeposit();
        robot.waitWhile(() -> !robot.deposit.isSampleDepositDone());


        // Depo 2
        robot.waitWhile(() -> !robot.clawIntake.isExtended());
        robot.clawIntake.grab(true);
        robot.waitWhile(() -> !robot.clawIntake.grabFinished());
        robot.drivetrain.goToPoint(
            new Pose2d(dx3, dy3, Math.toRadians(260)),
            false,
            true,
            1.0
        );
        robot.clawIntake.retract();
        robot.deposit.bufferUpwards();
        robot.deposit.prepareTransfer();
        robot.waitWhile(() -> !robot.clawIntake.isRetracted());

        robot.deposit.startTransfer();


        robot.waitWhile(() -> !robot.deposit.isSampleReady());
        robot.deposit.startSampleDeposit();
        robot.clawIntake.setIntakeTargetPos(robot.drivetrain.calcExtension(new Pose2d(dx3, dy3), new Pose2d(bx3, by3)));
        robot.clawIntake.extend();
        robot.waitWhile(() -> robot.drivetrain.isBusy() || !robot.deposit.isSampleUp());
        robot.deposit.finishSampleDeposit();
        robot.waitWhile(() -> !robot.deposit.isSampleDepositDone());

        // Depo 4
        robot.goToPoint(
            new Pose2d(dx3, dy3, Math.atan2(by3 - dy3, bx3 - dx3)),
            () -> !(robot.drivetrain.targetPoint.getDistanceFromPoint(robot.sensors.getOdometryPosition()) < 7),
            false,
            true,
            1.0
        );
        robot.waitWhile(() -> !robot.clawIntake.isExtended() || robot.drivetrain.isBusy());
        robot.clawIntake.grab(true);
        robot.waitWhile(() -> !robot.clawIntake.grabFinished());
        robot.drivetrain.goToPoint(
            new Pose2d(dx3, dy3, Math.toRadians(260)),
            false,
            true,
            1.0
        );
        robot.clawIntake.retract();
        robot.deposit.bufferUpwards();
        robot.deposit.prepareTransfer();
        robot.waitWhile(() -> !robot.clawIntake.isRetracted());

        robot.deposit.startTransfer();
        robot.waitWhile(() -> !robot.deposit.isSampleReady());

        robot.deposit.startSampleDeposit();
        robot.waitWhile(() -> robot.drivetrain.isBusy() || !robot.deposit.isSampleUp());
        robot.deposit.finishSampleDeposit();
        robot.waitWhile(() -> !robot.deposit.isSampleDepositDone());

        // Depo 5, 6, 7
        for(int cycle = 1; cycle <= 3; cycle++) {
            // Spline to submersible
            robot.drivetrain.setPath(new Spline(
                            new Pose2d(52.0, 24.0, -Math.PI / 2),
                            4
                    )
                            .addPoint(new Pose2d(30.5, 14.0, Math.PI))
            );

            // Pre-extension
            robot.setIntakeExtension(12.0);
            // Wait for last point to extend
            robot.waitWhile(() -> !(robot.drivetrain.getPath().poses.size() - 1 == robot.drivetrain.pathIndex));
            robot.clawIntake.extend();
            robot.waitWhile(() -> robot.drivetrain.isBusy());

            // phony vision
            double extension = 15.0; // Pretend this is the result of vision calculations
            robot.setIntakeExtension(extension);
            robot.clawIntake.extend();
            robot.waitWhile(() -> !robot.clawIntake.isExtensionAtTarget());

            // Grab at intended position
            robot.grab(true);
            robot.clawIntake.grab(true);
            robot.waitWhile(() -> !robot.clawIntake.grabFinished());

            // Go to bottom of high bucket to deposit
            robot.drivetrain.goToPoint(
                    new Pose2d(dx4, dy4, Math.atan2(dy4 - 72.0, dx4 - 72.0) + Math.PI),
                    true,
                    true,
                    1.0);
            robot.clawIntake.retract();
            robot.deposit.bufferUpwards();
            robot.deposit.prepareTransfer();
            robot.waitWhile(() -> !robot.clawIntake.isRetracted());

            // Transfer
            robot.deposit.startTransfer();
            robot.waitWhile(() -> !robot.deposit.isSampleReady());

            // Raise and deposit when at point
            robot.deposit.startSampleDeposit();
            robot.waitWhile(() -> robot.drivetrain.isBusy() || !robot.deposit.isSampleUp());
            robot.deposit.finishSampleDeposit();
            robot.waitWhile(() -> !robot.deposit.isSampleDepositDone());
        }
        RobotLog.i("Auto total time: " + (System.currentTimeMillis() - Globals.autoStartTime));
    }

//        moveToBelowBucket();
//        initialMove();
//        score();
//
//        if (enableg1) {
//            getGround(bx1, by);
//            moveToBelowBucket();
//            score();
//        }
//
//        if (enableg2) {
//            getGround(bx2, by);
//            moveToBelowBucket();
//            score();
//        }
//
//        if (enableg3) {
//            getGround(bx3, by);
//            moveToBelowBucket();
//            score();
//        }
//
//        if (enabler) {
//            goToTeleOpStart();
//        } else {
//            robot.waitWhile(() -> !robot.deposit.isRetractDone());
//        }

    /*public void doInitialization() {
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
        robot.deposit.startSampleDeposit();

        robot.clawIntake.setIntakeTargetPos(robot.drivetrain.calcExtension(new Pose2d(65, 54), new Pose2d(bx1, by)));

        double old = OldDrivetrain.finalTurnThreshold;
        //OldDrivetrain.finalTurnThreshold = 3;
        robot.goToPoint(
            new Pose2d(65, 54, Math.atan2(by - 54, bx1 - 65)), () ->
            !(robot.drivetrain.targetPoint.getDistanceFromPoint(robot.sensors.getOdometryPosition()) < 7),
            false,
            true,
            1.0
        );
        robot.clawIntake.extend();
        robot.waitWhile(() -> robot.drivetrain.isBusy() || !robot.deposit.isSampleUp());
        //OldDrivetrain.finalTurnThreshold = old;

        robot.deposit.finishSampleDeposit();
        robot.waitWhile(() -> !robot.deposit.isSampleDepositDone());
    }

    public void groundOne(){
        //robot.goToPointWithIntake(new Pose2d(bx1, by), null, true, false, false, 1.0, true);
        robot.waitWhile(() -> !robot.clawIntake.isExtended());
        robot.clawIntake.grab(true);
        robot.waitWhile(() -> !robot.clawIntake.grabFinished());
        robot.clawIntake.retract();
        robot.waitWhile(() -> !robot.clawIntake.isRetracted());

        robot.deposit.startTransfer();
    }

    public void initialMove(){
        robot.updateDepositHeights(false, true);
        robot.setNextState(Robot.NextState.DEPOSIT);

        robot.goToPoint(new Pose2d(62, 56, Math.PI * 17/12), null, false, false, 0.8);
    }

    public void moveToBelowBucket() {
        // robot current state, SAMPLE_READY
        robot.goToPoint(new Pose2d(sx, sy, sh), null, false, true, 0.9);
        robot.waitWhile(() -> !robot.clawIntake.isRetracted());

        // raise slides
        robot.updateDepositHeights(false, true);
        robot.setNextState(Robot.NextState.DEPOSIT);

        // wait for full raise
        robot.waitWhile(() -> !robot.deposit.isSampleUp());
    }

    public void score() {
        // move in (robot current state, DEPOSIT_BUCKET)
        robot.goToPoint(new Pose2d(sx, sy, sh), null, false, true, 0.9);

        // release sample
        robot.setNextState(Robot.NextState.DONE);
        //robot.waitFor(215);

        // wait for full retract
        robot.waitWhile(() ->  !robot.deposit.safeToMove());
    }

    public void getGround(double bx, double by) {
        robot.goToPointWithIntake(new Pose2d(bx, by, Math.toRadians(-90)), null, true, false, true, 0.9, true);

    }

    public void goToTeleOpStart() {
        // prepare for teleop
        robot.goToPoint(new Pose2d(fx1, fy1, fh1), null, false, false, 0.9);
        robot.deposit.holdSlides = true;
        robot.deposit.setDepositHeight(9.6);
        robot.goToPoint(new Pose2d(fx, fy, fh), null, true, true, 0.9);
        robot.waitWhile(() -> true);
    }*/
}