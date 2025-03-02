package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.utils.Globals;
import org.firstinspires.ftc.teamcode.utils.Pose2d;
import org.firstinspires.ftc.teamcode.utils.RunMode;

@Autonomous(name = "BucketPreloadBlueAuto", preselectTeleOp = "A. Teleop")
@Config
public class SamplePreloadBlueAuto extends LinearOpMode {
    private Robot robot;

    public static boolean enableg1 = true, enableg2 = true, enableg3 = true, enabler = true;

    public static double gx1 = 49.0, gy1 = 25.75;
    public static double gx2 = 60.0, gy2 = 25.75;
    public static double gx3 = 70.0, gy3 = 25.75;

    public static double fx1 = 36.0, fy1 = 12.0 , fh1 = Math.PI;
    public static double fx2 = 24.0, fy2 = 12.0, fh2 = Math.PI;

    public void runOpMode(){
        doInitialization();

        Globals.autoStartTime = System.currentTimeMillis();

        moveToBelowBucket();
        score();

        if (enableg1) {
            intakeGroundAt(gx1, gy1);
            moveToBelowBucket();
            score();
        }

        if (enableg2) {
            intakeGroundAt(gx2, gy2);
            moveToBelowBucket();
            score();
        }

        if (enableg3) {
            intakeGroundAt(gx3, gy3);
            moveToBelowBucket();
            score();
        }

        if (enabler) {
            goToTeleOpStart();
        } else {
            robot.waitWhile(() -> !robot.deposit.isRetractDone());
        }
    }

    public void doInitialization(){
        Globals.isRed = false;
        Globals.RUNMODE = RunMode.AUTO;
        Globals.hasSamplePreload = true;

        robot = new Robot(hardwareMap);
        robot.setAbortChecker(() -> !isStopRequested());

        robot.sensors.resetPosAndIMU();

        while (opModeInInit() && !isStopRequested()) {
            robot.sensors.setOdometryPosition(48.0 - Globals.TRACK_WIDTH / 2.0, 72.0 - Globals.TRACK_LENGTH / 2.0, Math.PI/2);
            robot.deposit.setDepositHeight(0.0);

            robot.update();
        }
    }

    public void moveToBelowBucket() {
        // robot current state, SAMPLE_READY
        robot.goToPoint(new Pose2d(55, 55, 5 * Math.PI/4), null, false, false, false, 0.8);
        robot.waitWhile(() -> !robot.intake.isRetracted());

        // raise slides
        robot.updateDepositHeights(false, true);
        robot.setNextState(Robot.NextState.DEPOSIT);

        // wait for full raise
        robot.waitWhile(() -> !robot.deposit.isSampleUp());
    }

    public void score() {
        // move in (robot current state, DEPOSIT_BUCKET)
        robot.goToPoint(new Pose2d(57, 57, 5 * Math.PI / 4), null, false, true, false, 0.8);

        // release sample
        robot.setNextState(Robot.NextState.DONE);
        //robot.waitFor(215);

        // wait for full retract
        robot.waitWhile(() ->  !robot.deposit.safeToMove());

        // back up
        //robot.goToPoint(new Pose2d(52, 52, 5 * Math.PI/4), null, false, true, 0.8);
    }

    public void intakeGroundAt(double gx, double gy){
        robot.goToPoint(new Pose2d(gx, gy), null, true, true, true, 0.8);

        robot.setNextState(Robot.NextState.INTAKE_SAMPLE);
        robot.setIntakeExtension(robot.drivetrain.getExtension());

        robot.waitWhile(() -> !robot.intake.isExtended());

        robot.waitFor(50);

        robot.grab(true);
        robot.waitWhile(() -> !robot.intake.grabFinished());

        robot.setNextState(Robot.NextState.DONE);
    }

    public void goToTeleOpStart() {
        // prepare for teleop
        robot.goToPoint(new Pose2d(fx1, fy1, fh1), null, false, false, false, 0.8);

        robot.goToPoint(new Pose2d(fx2, fy2, fh2), null, true, false, true, 0.8);
    }
}
