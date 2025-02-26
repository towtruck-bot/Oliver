//package org.firstinspires.ftc.teamcode.opmodes;
//
//import com.acmerobotics.dashboard.config.Config;
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//
//import org.firstinspires.ftc.teamcode.Robot;
//import org.firstinspires.ftc.teamcode.utils.Globals;
//import org.firstinspires.ftc.teamcode.utils.Pose2d;
//import org.firstinspires.ftc.teamcode.utils.RunMode;
//
//@Autonomous(name = "BucketPreloadBlueAuto", preselectTeleOp = "A. Teleop")
//@Config
//public class SamplePreloadBlueAuto extends LinearOpMode {
//    private Robot robot;
//
//    public static boolean enableg1 = true, enableg2 = true, enableg3 = true, enabler = true;
//
//    public static double gx = 60, gy = 48;
//    public static double g1h = -1.976, g1e = 15.95;
//    public static double g2h = -Math.PI/2, g2e = 13.7;
//    public static double g3h = -1.165, g3e = 15.95;
//    public static double fx = 24.0, fy = 12.0, fh = Math.PI;
//
//    public void runOpMode(){
//        doInitialization();
//
//        Globals.autoStartTime = System.currentTimeMillis();
//
//        moveToBelowBucket();
//        score();
//
//        if (enableg1) {
//            getGround(gx, gy, g1h, g1e);
//            moveToBelowBucket();
//            score();
//        }
//
//        if (enableg2) {
//            getGround(gx, gy, g2h, g2e);
//            moveToBelowBucket();
//            score();
//        }
//
//        if (enableg3) {
//            getGround(gx, gy, g3h, g3e);
//            moveToBelowBucket();
//            score();
//        }
//
//        if (enabler) {
//            goToTeleOpStart();
//        } else {
//            robot.waitWhile(() -> !robot.deposit.isRetractDone());
//        }
//    }
//
//    public void doInitialization(){
//        Globals.isRed = false;
//        Globals.RUNMODE = RunMode.AUTO;
//        Globals.hasSamplePreload = true;
//
//        robot = new Robot(hardwareMap);
//        robot.setAbortChecker(() -> !isStopRequested());
//
//        robot.sensors.resetPosAndIMU();
//
//        while (opModeInInit() && !isStopRequested()) {
//            robot.sensors.setOdometryPosition(48.0 - Globals.TRACK_WIDTH / 2.0, 72.0 - Globals.TRACK_LENGTH / 2.0, Math.PI/2);
//            robot.deposit.setDepositHeight(0.0);
//
//            robot.update();
//        }
//    }
//
//    public void moveToBelowBucket() {
//        // robot current state, SAMPLE_READY
//        robot.goToPoint(new Pose2d(55, 55, 5 * Math.PI/4), null, false, true, 0.8);
//        robot.waitWhile(() -> !robot.clawIntake.isRetracted());
//
//        // raise slides
//        robot.updateDepositHeights(false, true);
//        robot.setNextState(Robot.NextState.DEPOSIT);
//
//        // wait for full raise
//        robot.waitWhile(() -> !robot.deposit.isSampleUp());
//    }
//
//    public void score() {
//        // move in (robot current state, DEPOSIT_BUCKET)
//        robot.goToPoint(new Pose2d(57, 57, 5 * Math.PI/4), null, false, true, 0.8);
//
//        // release sample
//        robot.setNextState(Robot.NextState.DONE);
//        //robot.waitFor(215);
//
//        // wait for full retract
//        robot.waitWhile(() ->  !robot.deposit.safeToMove());
//
//        // back up
//        //robot.goToPoint(new Pose2d(52, 52, 5 * Math.PI/4), null, false, true, 0.8);
//    }
//
//    public void getGround(double gx, double gy, double gh, double ge){
//        // extend intake to desired length
//        robot.setNextState(Robot.NextState.INTAKE_SAMPLE);
//        robot.setIntakeExtension(ge - 5);
//
//        robot.goToPoint(new Pose2d(gx, gy, gh), null, true, true, 0.8);
//        robot.setIntakeExtension(ge);
//
//        robot.waitWhile(() -> !robot.clawIntake.isExtended());
//
//        // buffer time between extension and grab
//        robot.waitFor(300);
//
//        // grab
//        robot.grab(true);
//        robot.waitWhile(() -> !robot.clawIntake.grabFinished());
//
//        // retract
//        robot.setNextState(Robot.NextState.DONE);
//        //robot.waitWhile(() -> { return !robot.clawIntake.isRetracted(); });
//    }
//
//    public void goToTeleOpStart() {
//        // prepare for teleop
//        robot.goToPoint(new Pose2d(36, 12, Math.PI), null, false, false, 0.8);
//
//        robot.goToPoint(new Pose2d(fx, fy, fh), null, true, true, 0.8);
//    }
//}
