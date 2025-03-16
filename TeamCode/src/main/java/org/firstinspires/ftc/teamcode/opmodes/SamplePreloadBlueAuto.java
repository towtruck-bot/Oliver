package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.utils.Globals;
import org.firstinspires.ftc.teamcode.utils.LogUtil;
import org.firstinspires.ftc.teamcode.utils.Pose2d;
import org.firstinspires.ftc.teamcode.utils.RunMode;

@Autonomous(name = "SamplePreloadBlueAuto", preselectTeleOp = "A. Teleop")
@Config
public class SamplePreloadBlueAuto extends LinearOpMode {
    private Robot robot;

    public static boolean enableg1 = true, enableg2 = true, enableg3 = true, enabler = true;

    public static double by = 26, bx1 = 49.5, bx2 = 60, bx3 = 69.5;
    public static double fx1 = 36, fy1 = 12, fh1 = Math.PI;
    public static double fx = 20, fy = 12, fh = Math.PI;
    public static double sx = 57, sy = 57, sh = Math.toRadians(225);

    public void runOpMode(){
        doInitialization();

        Globals.autoStartTime = System.currentTimeMillis();

        moveToBelowBucket();
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
            robot.waitWhile(() -> !robot.deposit.isRetractDone());
        }
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
            robot.sensors.setOdometryPosition(48.0 - Globals.ROBOT_WIDTH / 2.0, 72.0 - Globals.ROBOT_FORWARD_LENGTH, Math.PI/2);
            robot.updateDepositHeights(false, true);

            robot.update();
        }
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

        // back up
        //robot.goToPoint(new Pose2d(52, 52, 5 * Math.PI/4), null, false, true, 0.8);
    }

    public void getGround(double bx, double by) {
/*
        // extend intake to desired length
        robot.setNextState(Robot.NextState.INTAKE_SAMPLE);
        robot.setIntakeExtension(ge - 5);

        robot.goToPoint(new Pose2d(gx, gy, gh), null, true, true, 0.9);
        robot.clawIntake.setClawRotation(Math.toRadians(-90) - gh);
        robot.setIntakeExtension(ge);

        robot.waitWhile(() -> !robot.clawIntake.isExtended());

        // buffer time between extension and grab
        robot.waitFor(300);

        // grab
        robot.grab(true);
        robot.waitWhile(() -> !robot.clawIntake.grabFinished());

        // retract
        robot.setNextState(Robot.NextState.DONE);
 */
        robot.goToPointWithIntake(new Pose2d(bx, by, Math.toRadians(-90)), null, true, false, true, 0.9, true);
        //robot.waitWhile(() -> { return !robot.clawIntake.isRetracted(); });
    }

    public void goToTeleOpStart() {
        // prepare for teleop
        robot.goToPoint(new Pose2d(fx1, fy1, fh1), null, false, false, 0.9);
        robot.deposit.holdSlides = true;
        robot.deposit.setDepositHeight(9.6);
        robot.goToPoint(new Pose2d(fx, fy, fh), null, true, true, 0.9);
        robot.waitWhile(() -> true);
    }
}