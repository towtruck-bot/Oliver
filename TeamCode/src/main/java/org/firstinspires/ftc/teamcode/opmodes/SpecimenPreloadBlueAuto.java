package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.utils.Globals;
import org.firstinspires.ftc.teamcode.utils.Pose2d;
import org.firstinspires.ftc.teamcode.utils.RunMode;

@Autonomous(name = "SpecimenPreloadBlueAuto", preselectTeleOp = "A. Teleop")
@Config
public class SpecimenPreloadBlueAuto extends LinearOpMode {
    private Robot robot;

    public static double g1x = -46.0, g2x = -52.0, g3x = -72.0 + Globals.TRACK_WIDTH / 2.0;
    public static double ypre = 23.0 - Globals.TRACK_LENGTH / 2.0, ypush = 63.0 - Globals.TRACK_LENGTH / 2.0;
    public static double setupx = -36.0, setupy = 60.0;
    public static double shift = 2.0, baseScoreX = -Globals.TRACK_WIDTH/ 2.0, baseScoreY = 48.0 - Globals.TRACK_LENGTH / 2.0, scoreY = 24.0 + Globals.TRACK_LENGTH;

    public void runOpMode(){
        doInitialization();

        Globals.autoStartTime = System.currentTimeMillis();

        score(.0);
        move3Ground();

        for(int i = 1; i <= 5; i++){
            grabAndSetUp(i * shift);
            score(i * shift);
        }
    }

    public void doInitialization(){
        Globals.isRed = false;
        Globals.RUNMODE = RunMode.AUTO;
        Globals.hasSpecimenPreload = true;

        robot = new Robot(hardwareMap);
        robot.setAbortChecker(() -> !isStopRequested());

        robot.sensors.resetPosAndIMU();

        while (opModeInInit() && !isStopRequested()) {
            robot.sensors.setOdometryPosition( -Globals.TRACK_WIDTH / 2.0, 72.0 - Globals.TRACK_LENGTH / 2.0, Math.PI/2);
            robot.deposit.setDepositHeight(0.0);

            robot.update();
        }
    }

    public void score(double offset){
        // Extend deposit slides into deposit position
        robot.setNextState(Robot.NextState.DEPOSIT);
        robot.waitWhile(() -> !robot.deposit.slides.inPosition(0.5) && !robot.deposit.arm.inPosition());
        // ^Wait for fully raised slides and arm at correct angle^

        // Ram forward
        robot.goToPoint(new Pose2d(baseScoreX + offset, scoreY, Math.PI/2), null, false, false, 1.0);

        // Release + backup
        robot.setNextState(Robot.NextState.DONE);
        robot.goToPoint(new Pose2d(baseScoreX, baseScoreY, -Math.PI/2), null, false, false, 0.8);
    }

    public void move3Ground(){
        // Pre-position ground1
        // TODO: Switch to a spline once they are re-tuned
        robot.goToPoint(new Pose2d(-33.0, 36.0, Math.PI), null, false, false, 0.8);
        robot.goToPoint(new Pose2d(-33.0, ypre, Math.PI / 2.0), null, false, false, 0.8);
        robot.goToPoint(new Pose2d(g1x, ypre, Math.PI / 2.0), null, true, true, 0.8);

        // Deliver ground1
        robot.goToPoint(new Pose2d(g1x, ypush, Math.PI / 2.0), null, true, false, 0.8);

        // Pre-position ground2
        robot.goToPoint(new Pose2d(g2x + 3.0, ypre, Math.PI / 2.0), null, false, false, 0.8);
        robot.goToPoint(new Pose2d(g2x, ypre, Math.PI / 2.0), null, true, false, 0.8);

        // Deliver ground2
        robot.goToPoint(new Pose2d(g2x, ypush, Math.PI / 2.0), null, true, false, 0.8);

        // Pre-position ground3
        robot.goToPoint(new Pose2d(g3x + 3.0, ypre, Math.PI / 2.0), null, false, false, 0.8);
        robot.goToPoint(new Pose2d(g3x, ypre, Math.PI / 2), null, true, false, 0.8);

        // Deliver ground3
        robot.goToPoint(new Pose2d(g3x, ypush, Math.PI / 2.0), null, true, false, 0.8);
    }

    public void grabAndSetUp(double offset){
        // Prepare to grab
        robot.setNextState(Robot.NextState.GRAB_SPECIMEN);
        robot.goToPoint(new Pose2d(setupx, setupy, Math.PI), null, true, true, 0.8);

        // Grab
        robot.setNextState(Robot.NextState.GRAB_SPECIMEN);
        robot.waitWhile(() -> robot.deposit.isSpecimenReady());

        // Move to setup
        // Offset is used such that the specimen wont clip on each other
        robot.goToPoint(new Pose2d(baseScoreX + offset, baseScoreY, Math.PI), null, true, false, 0.8);
    }
}
