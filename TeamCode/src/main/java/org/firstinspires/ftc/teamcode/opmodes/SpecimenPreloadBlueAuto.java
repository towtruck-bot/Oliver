package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.subsystems.drive.Drivetrain;
import org.firstinspires.ftc.teamcode.utils.Globals;
import org.firstinspires.ftc.teamcode.utils.LogUtil;
import org.firstinspires.ftc.teamcode.utils.Pose2d;
import org.firstinspires.ftc.teamcode.utils.RunMode;

@Autonomous(name = "SpecimenPreloadBlueAuto", preselectTeleOp = "A. Teleop")
@Config
public class SpecimenPreloadBlueAuto extends LinearOpMode {
    private Robot robot;

    public static boolean enablePreload = true, enableGround = true, enableScore = true;

    public static double g1x = -47, g2x = -57, g3x = -63.5;
    public static double yPre = 16, yPush = 55;
    public static double setupX = -37, setupY = 64.3;
    public static double shift = 3.742, baseScoreX = -2, baseScoreY = 42, scoreY = 25.85;

    public static double[] targetX = new double[] {-3.5, -1, 1.5, 4, 6.5};

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
            for (int i = 0; i < 4; i++) {
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
            //robot.deposit.setDepositHeight(0.0);

            robot.update();
        }
    }

    public void score(double offset) {
        // Extend deposit slides into deposit position
        robot.setNextState(Robot.NextState.DEPOSIT);
        //robot.waitWhile(() -> !robot.deposit.readyToRam());
        // ^Wait for fully raised slides and arm at correct angle^

        robot.drivetrain.slidesUp = true;
        Drivetrain.xThreshold = 1.75;
        Drivetrain.yThreshold = 1.75;
        Drivetrain.turnThreshold = 5;

        // Ram forward
        robot.goToPoint(new Pose2d(offset, scoreY, Math.PI/2), null, false, false, 1.0);
        robot.waitFor(150);

        Drivetrain.xThreshold = 4;
        Drivetrain.yThreshold = 4;
        Drivetrain.turnThreshold = 4;

        // Release + backup
        robot.setNextState(Robot.NextState.DONE);
        robot.goToPoint(new Pose2d(baseScoreX, baseScoreY, Math.PI/2), null, false, false, 1.0);

        robot.drivetrain.slidesUp = false;
    }

    public void move3Ground() {
        // Pre-position ground1
        // TODO: Switch to a spline once they are re-tuned
        Drivetrain.xThreshold = 2;
        Drivetrain.yThreshold = 2;
        Drivetrain.turnThreshold = 4;

        robot.drivetrain.slidesUp = false;

        robot.goToPoint(new Pose2d(-36.0, baseScoreY, Math.PI / 2.0), null, false, false, 1.0);
        robot.goToPoint(new Pose2d(-36.0, yPre, Math.PI / 2.0), null, false, false, 1.0);
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
        robot.goToPoint(new Pose2d(g3x + 6, yPush - 3, Math.PI / 2.0), null, false, true, 1.0);
    }

    public void grabAndSetUp(double offset) {
        // Pre-position
        Drivetrain.xThreshold = 4;
        Drivetrain.yThreshold = 4;
        Drivetrain.turnThreshold = 4;
        robot.drivetrain.slidesUp = false;
        robot.goToPoint(new Pose2d(setupX + 3.0, setupY - 6.0, Math.PI / 2.0), null, false, false, 1.0);

        Drivetrain.xThreshold = 1.75;
        Drivetrain.yThreshold = 1.75;
        Drivetrain.turnThreshold = 4;
        // Prepare to grab
        robot.setNextState(Robot.NextState.GRAB_SPECIMEN);
        robot.goToPoint(new Pose2d(setupX, setupY, Math.PI / 2.0), null, true, true, 0.8);

        // Grab
        robot.setNextState(Robot.NextState.GRAB_SPECIMEN);
        //robot.waitWhile(() -> !robot.deposit.isSpecimenReady());

        robot.setNextState(Robot.NextState.DEPOSIT);
        robot.drivetrain.slidesUp = true;

        Drivetrain.xThreshold = 4;
        Drivetrain.yThreshold = 4;
        Drivetrain.turnThreshold = 4;
        // Move to setup
        // Offset is used such that the specimen wont clip on each other
        robot.goToPoint(new Pose2d(offset, baseScoreY, Math.PI / 2.0), null, false, false, 1.0);
    }

    public void park() {
        robot.goToPoint(new Pose2d(setupX - 6.0, setupY, Math.PI / 2.0), null, false, true, 1.0);
    }
}
