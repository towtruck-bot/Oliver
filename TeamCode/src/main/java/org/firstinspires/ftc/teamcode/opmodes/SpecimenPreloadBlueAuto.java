package org.firstinspires.ftc.teamcode.opmodes;

import android.util.Log;

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

    public static double ox = -56.0, oy = 64.0;
    public static double setupX = -38, setupY = 63.9;
    public static double baseScoreX = -5.9, baseScoreY = 41.45, scoreY = 28.75, backUpX = -8, backUpY = 40.0;

    public static double[] targetX = new double[] {0.0, 2.5, 5.0, 7.5, 10.0};

    public void runOpMode(){
        doInitialization();

        Globals.autoStartTime = System.currentTimeMillis();

        if(enablePreload){
            score(-Globals.ROBOT_WIDTH / 2.0);
        }

        if(enableGround){
            move3Ground();
        }

        if(enableScore){
            for(int i = 0; i < 5; i++){
                // go into park
                if(System.currentTimeMillis() - Globals.autoStartTime <= 4000){
                    break;
                }

                grabAndSetUp(targetX[i]);
                score(targetX[i]);
            }
        }

        park();
    }

    public void doInitialization(){
        Globals.isRed = false;
        Globals.RUNMODE = RunMode.AUTO;
        Globals.hasSpecimenPreload = true;
        Globals.hasSamplePreload = false;

        robot = new Robot(hardwareMap);
        robot.setAbortChecker(() -> !isStopRequested());
        LogUtil.init();

        robot.sensors.resetPosAndIMU();

        while (opModeInInit() && !isStopRequested()) {
            robot.sensors.setOdometryPosition( -Globals.ROBOT_WIDTH / 2.0, 72.0 - Globals.ROBOT_FORWARD_LENGTH, Math.PI/2);
            robot.deposit.setDepositHeight(0.0);
            robot.updateDepositHeights(true, true);

            robot.update();
        }
    }

    public void score(double offset){
        // Extend deposit slides into deposit position
        robot.setNextState(Robot.NextState.DEPOSIT);
        robot.waitWhile(() -> !robot.deposit.readyToRam());
        // ^Wait for fully raised slides and arm at correct angle^

        // Ram forward
        robot.goToPoint(new Pose2d(offset, scoreY, Math.PI/2), null, false, false, 0.85);
        robot.waitFor(75);

        // Release + backup
        robot.setNextState(Robot.NextState.DONE);
        robot.goToPoint(new Pose2d(backUpX, backUpY, Math.PI), null, false, false, 0.95);
    }

    public static double s1x = -49.5, s2x = -60.0, s3x = -69.5;
    public static double sy = 26.0;

    public void move3Ground(){
        robot.goToPointWithIntake(new Pose2d(s1x, sy, -Math.PI / 2), null, true, false, true, 0.95, true);
        robot.setNextState(Robot.NextState.DONE);
        robot.goToPointWithIntake(new Pose2d(ox, oy, Math.PI / 2), null, true, false, true, 0.95, false);
        robot.setNextState(Robot.NextState.DONE);

        robot.goToPointWithIntake(new Pose2d(s2x, sy, -Math.PI / 2), null, true, false, true, 0.95, true);
        robot.setNextState(Robot.NextState.DONE);
        robot.goToPointWithIntake(new Pose2d(ox, oy, Math.PI / 2), null, true, false, true, 0.95, false);
        robot.setNextState(Robot.NextState.DONE);

        robot.goToPointWithIntake(new Pose2d(s3x, sy, -Math.PI / 2), null, true, false, true, 0.95, true);
        robot.setNextState(Robot.NextState.DONE);
        robot.goToPointWithIntake(new Pose2d(ox, oy, Math.PI / 2), null, true, false, true, 0.95, false);
        robot.setNextState(Robot.NextState.DONE);
    }

    public void grabAndSetUp(double offset){
        // Pre-position
        robot.goToPoint(new Pose2d(setupX, setupY - 6.0, Math.PI / 2.0), null, false, false, 0.95);

        // Prepare to grab
        robot.setNextState(Robot.NextState.GRAB_SPECIMEN);
        robot.goToPoint(new Pose2d(setupX, setupY, Math.PI / 2.0), null, true, true, 0.8);

        // Grab
        robot.setNextState(Robot.NextState.GRAB_SPECIMEN);
        robot.waitWhile(() -> !robot.deposit.isSpecimenReady());

        // Move to setup
        // Offset is used such that the specimen wont clip on each other
        robot.goToPoint(new Pose2d(offset, baseScoreY, Math.PI / 2.0), null, false, false, 0.95);
    }

    public void park(){
        robot.goToPoint(new Pose2d(setupX - 6.0, setupY, Math.PI / 2.0), null, false, true, 1.0);
    }
}