package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.subsystems.deposit.nDeposit;
import org.firstinspires.ftc.teamcode.subsystems.intake.nClawIntake;
import org.firstinspires.ftc.teamcode.utils.Globals;
import org.firstinspires.ftc.teamcode.utils.LogUtil;
import org.firstinspires.ftc.teamcode.utils.Pose2d;
import org.firstinspires.ftc.teamcode.utils.RunMode;

@Autonomous(name = "SamplePreloadBlueAuto", preselectTeleOp = "A. Teleop")
@Config
public class SamplePreloadBlueAuto extends LinearOpMode {
    private Robot robot;

    // Block positions
    public static double by1 = 21.75, bx1 = 49.5, bx2 = 59.5, by2 = 22.25, bx3 = 69, by3 = 23.75;

    // GP depo positions
    public static double dx1 = 62.3, dy1 = 52.5;
    public static double dx2 = 64.5, dy2 = 52;
    public static double dx3 = 66, dy3 = 53;

    public void runOpMode(){
        Globals.isRed = false;
        Globals.RUNMODE = RunMode.AUTO;
        Globals.hasSamplePreload = true;
        Globals.hasSpecimenPreload = false;

        robot = new Robot(hardwareMap);
        robot.setAbortChecker(() -> !isStopRequested());
        LogUtil.init();

        robot.sensors.resetPosAndIMU();

        robot.ndeposit.state = nDeposit.State.HOLD;
        robot.nclawIntake.useCamera(true);
        robot.nclawIntake.setUseManualGrab(false);
        robot.nclawIntake.setAutoGrab(false);
        while (opModeInInit() && !isStopRequested()) {
            //robot.sensors.setOdometryPosition(48.0 - Globals.ROBOT_WIDTH / 2.0, 72.0 - Globals.ROBOT_FORWARD_LENGTH, Math.PI/2);
            robot.sensors.setOdometryPosition(48.0 - Globals.ROBOT_REVERSE_LENGTH, 72.0 - Globals.ROBOT_WIDTH / 2, Math.PI);
            robot.updateDepositHeights(false, true);

            robot.update();
        }

        Globals.autoStartTime = System.currentTimeMillis();

        // Preload
        robot.ndeposit.startSampleDeposit();
        robot.goToPoint(
            new Pose2d(dx1, dy1, Math.toRadians(250)),
            () -> !(robot.drivetrain.targetPoint.getDistanceFromPoint(robot.sensors.getOdometryPosition()) < 8),
            false,
            true,
            1.0
        );
        //robot.nclawIntake.setIntakeLength(robot.drivetrain.calcExtension(new Pose2d(dx1, dy1), new Pose2d(bx1, by1)));
        robot.nclawIntake.setKnownIntakePose(new Pose2d(bx1, by1, Math.PI / 2));
        robot.nclawIntake.setAutoGrab(false);
        robot.nclawIntake.extend();
        robot.waitWhile(() -> robot.drivetrain.isBusy() || !robot.ndeposit.isDepositReady());

        robot.ndeposit.deposit();
        robot.waitWhile(() -> !robot.ndeposit.isDepositFinished());

        // Depo 1
        robot.waitWhile(() -> !robot.nclawIntake.isExtended());
        robot.nclawIntake.setAutoGrab(true);
        robot.waitWhile(() -> !robot.nclawIntake.hasSample());
        // Go to under bucket
        robot.drivetrain.goToPoint(
            new Pose2d(dx2, dy2, Math.atan2(by2 - dy2, bx2 - dx2)),
            false,
            true,
            1.0
        );
        robot.waitWhile(() -> !robot.nclawIntake.isTransferReady() || !robot.ndeposit.isTransferReady());
        robot.ndeposit.startSampleDeposit(); // This effectively buffers it
        robot.nclawIntake.finishTransfer();
        robot.ndeposit.finishTransfer();
        //robot.nclawIntake.setIntakeLength(robot.drivetrain.calcExtension(new Pose2d(dx2, dy2), new Pose2d(bx2, by2)));
        robot.nclawIntake.setKnownIntakePose(new Pose2d(bx2, by2, Math.PI / 2));
        robot.nclawIntake.setAutoGrab(false);
        robot.nclawIntake.extend();

        robot.waitWhile(() -> robot.drivetrain.isBusy() || !robot.ndeposit.isDepositReady());
        robot.ndeposit.deposit();
        robot.waitWhile(() -> !robot.ndeposit.isDepositFinished());


        // Depo 2
        robot.waitWhile(() -> !robot.nclawIntake.isExtended());
        robot.nclawIntake.setAutoGrab(true);
        robot.waitWhile(() -> !robot.nclawIntake.hasSample());
        // Go to under bucket
        robot.drivetrain.goToPoint(
            new Pose2d(dx3, dy3, Math.toRadians(278)),
            false,
            true,
            1.0
        );
        robot.waitWhile(() -> !robot.nclawIntake.isTransferReady() || !robot.ndeposit.isTransferReady());
        robot.ndeposit.startSampleDeposit(); // This effectively buffers it
        robot.nclawIntake.finishTransfer();
        robot.ndeposit.finishTransfer();
        robot.nclawIntake.setAutoGrab(false);
        robot.nclawIntake.useCamera(false);
        robot.nclawIntake.setUseManualGrab(true);
        robot.nclawIntake.setGrab(false);
        robot.nclawIntake.setTargetType(nClawIntake.Target.GLOBAL);
        robot.nclawIntake.setTargetPose(new Pose2d(bx3, by3, Math.PI / 2));
        //robot.nclawIntake.setKnownIntakePose(new Pose2d(bx3, by3, Math.PI / 2));
        //robot.nclawIntake.setAutoGrab(false);
        robot.nclawIntake.extend();

        robot.waitWhile(() -> robot.drivetrain.isBusy() || !robot.ndeposit.isDepositReady());
        robot.ndeposit.deposit();
        robot.waitWhile(() -> !robot.ndeposit.isDepositFinished());


        // Depo 3
        robot.waitWhile(() -> !robot.nclawIntake.isExtended());
        robot.nclawIntake.setGrab(true);
        //robot.nclawIntake.setAutoGrab(true);
        robot.waitWhile(() -> !robot.nclawIntake.hasSample());
        // Go to under bucket
        robot.drivetrain.goToPoint(
                new Pose2d(dx3, dy3, Math.toRadians(260)),
                false,
                true,
                1.0
        );
        robot.waitWhile(() -> !robot.nclawIntake.isTransferReady() || !robot.ndeposit.isTransferReady());
        robot.ndeposit.startSampleDeposit();
        robot.nclawIntake.finishTransfer();
        robot.ndeposit.finishTransfer();
        robot.nclawIntake.setAutoGrab(false);

        robot.waitWhile(() -> robot.drivetrain.isBusy() || !robot.ndeposit.isDepositReady());
        robot.ndeposit.deposit();
        robot.drivetrain.goToPoint(
                new Pose2d(dx3, dy3 + 5, Math.toRadians(260)),
                false,
                true,
                1.0
        );
        robot.waitWhile(() -> !robot.ndeposit.isDepositFinished());

        // TODO: Remember to set auto grab to true, set use camera to true, and DONT use grab

        // Depo 3
        /*robot.goToPoint(
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
        robot.waitWhile(() -> !robot.ndeposit.isSampleDepositDone());*/

        RobotLog.i("Auto total time: " + (System.currentTimeMillis() - Globals.autoStartTime));

        // Depo 5, 6, 7
        /*for(int cycle = 1; cycle <= 3; cycle++) {
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
//            robot.drivetrain.goToPoint(
//                    new Pose2d(dx4, dy4, Math.atan2(dy4 - 72.0, dx4 - 72.0) + Math.PI),
//                    true,
//                    true,
//                    1.0);
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
        }*/
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
//            robot.waitWhile(() -> !robot.ndeposit.isRetractDone());
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
    }*/
}