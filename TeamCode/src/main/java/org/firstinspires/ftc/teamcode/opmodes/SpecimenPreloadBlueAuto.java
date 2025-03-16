package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.utils.Globals;
import org.firstinspires.ftc.teamcode.utils.LogUtil;
import org.firstinspires.ftc.teamcode.utils.Pose2d;
import org.firstinspires.ftc.teamcode.utils.RunMode;

@Autonomous(name = "SpecimenPreloadBlueAuto", preselectTeleOp = "A. Teleop")
@Config
public class SpecimenPreloadBlueAuto extends LinearOpMode {
    private Robot robot;

    public static boolean enablePreload = true, enableGround = true, enableScore = true;

    public static double g1x = -46.0, g2x = -56.0, g3x = -63.5;
    public static double yPre = 15.0, yPush = 57.0;
    public static double setupX = -36.5, setupY = 64;
    public static double shift = 3.742, baseScoreX = -5.9, baseScoreY = 42, scoreY = 28.75;

    public static double[] targetX = new double[] {0.0, 2.5, 5.0, 7.5, 10.0};

    public void runOpMode() {
        doInitialization();

        Globals.autoStartTime = System.currentTimeMillis();

        if (enablePreload) {
            score(-Globals.ROBOT_WIDTH / 2.0);
        }

        if (enableGround) {
            move3Ground();
        }

        if (enableScore) {
            for (int i = 0; i < 5; i++) {
                // go into park
                if (System.currentTimeMillis() - Globals.autoStartTime <= 5000) {
                    break;
                }

                grabAndSetUp(targetX[i]);
                score(targetX[i]);
            }
        }

        park();
    }

    public void doInitialization() {
        Globals.isRed = false;
        Globals.RUNMODE = RunMode.AUTO;
        Globals.hasSamplePreload = false;
        Globals.hasSpecimenPreload = true;

        robot = new Robot(hardwareMap);
        robot.setAbortChecker(() -> !isStopRequested());
        LogUtil.init();

        robot.sensors.resetPosAndIMU();

        while (opModeInInit() && !isStopRequested()) {
            robot.sensors.setOdometryPosition( -Globals.ROBOT_WIDTH / 2.0, 72.0 - Globals.ROBOT_FORWARD_LENGTH, Math.PI/2);
            robot.deposit.setDepositHeight(0.0);

            robot.update();
        }
    }

    public void score(double offset) {
        // Extend deposit slides into deposit position
        robot.setNextState(Robot.NextState.DEPOSIT);
        robot.waitWhile(() -> !robot.deposit.readyToRam());
        // ^Wait for fully raised slides and arm at correct angle^

        // Ram forward
        robot.goToPoint(new Pose2d(offset, scoreY, Math.PI/2), null, false, false, 1.0);
        robot.waitFor(150);

        // Release + backup
        robot.setNextState(Robot.NextState.DONE);
        robot.goToPoint(new Pose2d(baseScoreX, baseScoreY, Math.PI/2), null, false, false, 1.0);
    }

    public void move3Ground() {
        // Pre-position ground1
        // TODO: Switch to a spline once they are re-tuned
        robot.goToPoint(new Pose2d(-38.0, 38.0, Math.PI / 2.0), null, false, false, 1.0);
        robot.goToPoint(new Pose2d(-38.0, yPre, Math.PI / 2.0), null, false, false, 1.0);
        robot.goToPoint(new Pose2d(g1x, yPre, Math.PI / 2.0), null, false, false, 1.0);

        // Deliver ground1
        robot.goToPoint(new Pose2d(g1x, yPush, Math.PI / 2.0), null, false, false, 1.0);

        // Pre-position ground2
        robot.goToPoint(new Pose2d(g1x, yPre, Math.PI / 2.0), null, false, false, 1.0);
        robot.goToPoint(new Pose2d(g2x, yPre, Math.PI / 2.0), null, false, false, 1.0);

        // Deliver ground2
        robot.goToPoint(new Pose2d(g2x, yPush, Math.PI / 2.0), null, false, false, 1.0);

        // Pre-position ground3
        robot.goToPoint(new Pose2d(g2x, yPre, Math.PI / 2.0), null, false, false, 1.0);
        robot.goToPoint(new Pose2d(g3x, yPre, Math.PI / 2.0), null, false, false, 1.0);

        // Deliver ground3
        robot.goToPoint(new Pose2d(g3x, yPush, Math.PI / 2.0), null, false, true, 1.0);
    }

    public void grabAndSetUp(double offset) {
        // Pre-position
        robot.goToPoint(new Pose2d(setupX + 3.0, setupY - 6.0, Math.PI / 2.0), null, false, false, 1.0);

        // Prepare to grab
        robot.setNextState(Robot.NextState.GRAB_SPECIMEN);
        robot.goToPoint(new Pose2d(setupX, setupY, Math.PI / 2.0), null, true, true, 0.8);

        // Grab
        robot.setNextState(Robot.NextState.GRAB_SPECIMEN);
        robot.waitWhile(() -> !robot.deposit.isSpecimenReady());

        robot.setNextState(Robot.NextState.DEPOSIT);

        // Move to setup
        // Offset is used such that the specimen wont clip on each other
        robot.goToPoint(new Pose2d(offset, baseScoreY, Math.PI / 2.0), null, false, false, 1.0);
    }

    public void park() {
        robot.goToPoint(new Pose2d(setupX - 6.0, setupY, Math.PI / 2.0), null, false, true, 1.0);
    }
}
