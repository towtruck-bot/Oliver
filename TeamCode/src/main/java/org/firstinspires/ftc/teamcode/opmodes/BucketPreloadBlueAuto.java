package org.firstinspires.ftc.teamcode.opmodes;

import android.util.Log;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.utils.Globals;
import org.firstinspires.ftc.teamcode.utils.Pose2d;
import org.firstinspires.ftc.teamcode.utils.RunMode;

@Autonomous(name = "BucketPreloadBlueAuto", preselectTeleOp = "nTeleop")
@Config
public class BucketPreloadBlueAuto extends LinearOpMode {
    private Robot robot;

    public static boolean enableg1 = true, enableg2 = true, enableg3 = true, enabler = true;

    public static double g1x = 49, g1y = 48, g1h = -Math.PI/2, g1e = 13.75;
    public static double g2x = 60, g2y = 48, g2h = -Math.PI/2, g2e = 13.75;
    public static double g3x = 60, g3y = 48, g3h = -1.165, g3e = 16;
    public static double fx = 24.0, fy = 12.0, fh = Math.PI;

//    public static double[] gx = {48, 60, 60};
//    public static double[] gy = {48, 48, 48};
//    public static double[] gh = {-Math.PI/2, -Math.PI/2, -Math.PI/3};
//    public static double[] ge = {13, 13, 16};

    public void runOpMode(){
        doInitialization();

        Globals.autoStartTime = System.currentTimeMillis();

        moveToBelowBucket();
        score();

        if (enableg1) {
            getGround(g1x, g1y, g1h, g1e);
            moveToBelowBucket();
            score();
        }

        if (enableg2) {
            getGround(g2x, g2y, g2h, g2e);
            moveToBelowBucket();
            score();
        }

        if (enableg3) {
            getGround(g3x, g3y, g3h, g3e);
            moveToBelowBucket();
            score();
        }

        if (enabler) {
            goToTeleOpStart();
        } else {
            robot.waitWhile(() -> { return !robot.deposit.isRetractDone(); });
        }
    }

    public void doInitialization(){
        Globals.isRed = false;
        Globals.RUNMODE = RunMode.AUTO;
        Globals.hasSamplePreload = true;

        robot = new Robot(hardwareMap);
        robot.setAbortChecker(() -> { return !isStopRequested(); });

        robot.sensors.resetPosAndIMU();

        while (opModeInInit() && !isStopRequested()) {
            robot.sensors.setOdometryPosition(48.0 - Globals.TRACK_WIDTH / 2.0, 72.0 - Globals.TRACK_LENGTH / 2.0, Math.PI/2);
            robot.deposit.setDepositHeight(0.0);

            robot.update();
        }
    }

    public void moveToBelowBucket() {
        // robot current state, SAMPLE_READY
        robot.goToPoint(new Pose2d(55, 55, 5 * Math.PI/4), null, false, true, 0.8);
        robot.waitWhile(() -> { return !robot.clawIntake.isRetracted(); });

        // raise slides
        robot.updateDepositHeights(false, true);
        robot.setNextState(Robot.NextState.DEPOSIT);

        // wait for full raise
        robot.waitWhile(() -> { return !robot.deposit.isSampleUp(); });
    }

    public void score() {
        // move in (robot current state, DEPOSIT_BUCKET)
        robot.goToPoint(new Pose2d(58, 58, 5 * Math.PI/4), null, false, true, 0.8);

        // release sample
        robot.setNextState(Robot.NextState.DONE);
        robot.waitFor(200);

        // back up
        robot.goToPoint(new Pose2d(55, 55, 5 * Math.PI/4), null, false, true, 0.8);

        // wait for full retract
        //robot.waitWhile(() -> { return !robot.deposit.isRetractDone(); });
    }
/*
    public void getGround(int index) {
        robot.goToPoint(new Pose2d(gx[index], gy[index], gh[index]), null, true, true, 0.8);

        //intake sample
        robot.setNextState(Robot.NextState.INTAKE_SAMPLE);
        robot.setIntakeExtension(ge[index]);
        robot.waitWhile(() -> { return !robot.clawIntake.isExtended(); });
        robot.waitFor(500);

        // this will cause clawIntake FSM to lower and grab. previous length was manually measured such that the claw would flip down on the sample
        robot.grab(true);
        robot.waitWhile(() -> { return !robot.clawIntake.grabFinished(); });

        robot.setNextState(Robot.NextState.DONE);
        robot.waitWhile(() -> { return !robot.clawIntake.isRetracted(); });
    }
*/
    public void getGround(double gx, double gy, double gh, double ge){
        robot.goToPoint(new Pose2d(gx, gy, gh), null, true, true, 0.8);

        // extend intake to desired length
        robot.setNextState(Robot.NextState.INTAKE_SAMPLE);
        robot.setIntakeExtension(ge);
        robot.waitWhile(() -> { return !robot.clawIntake.isExtended() /*&& !robot.deposit.isRetractDone()*/; });

        // buffer time between extension and grab
        robot.waitFor(250);

        // grab
        robot.grab(true);
        robot.waitWhile(() -> { return !robot.clawIntake.grabFinished(); });

        // retract
        robot.setNextState(Robot.NextState.DONE);
        //robot.waitWhile(() -> { return !robot.clawIntake.isRetracted(); });
    }


    public void goToTeleOpStart() {
        // prepare for teleop
        robot.goToPoint(new Pose2d(36, 12, Math.PI), null, false, false, 0.8);

        robot.goToPoint(new Pose2d(fx, fy, fh), null, true, true, 0.8);
    }
}
