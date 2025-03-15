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

    public static double gx = -60.0, gy = 52.25, cy = 56.0;
    public static double g1e = 15.95, g1h = -1.976;
    public static double g2e = 13.7, g2h = -Math.PI / 2;
    public static double g3e = 15.95, g3h = -1.165;

    public static double setupX = -36.5, setupY = 63.9;
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

    public void move3Ground(){
//        robot.goToPoint(new Pose2d(-24.0, gy, -Math.PI / 2), null, true, false, 0.95);
//        robot.goToPoint(new Pose2d(gx, gy, -Math.PI / 2), null, false, true, 0.95);

        robot.goToPoint(new Pose2d(gx, gy, -Math.PI / 2), null, false, true, 0.95);

        Log.i("AUTO", "start 1");
        pickUp(g2e, g2h);
        chuckOut();
        Log.i("AUTO", "end 1");

        Log.i("AUTO", "start 2");
        pickUp(g1e, g1h);
        chuckOut();
        Log.i("AUTO", "end 2");

        Log.i("AUTO", "start 3");
        pickUp(g3e, g3h);
        chuckOut();
        Log.i("AUTO", "end 3");
    }

    public void pickUp(double ge, double gh){
        robot.goToPoint(new Pose2d(gx, gy, gh), null, true, true, 0.95);

        robot.setIntakeExtension(ge);
        robot.setNextState(Robot.NextState.INTAKE_SAMPLE);

        robot.waitWhile(() -> !robot.clawIntake.isExtended());

        // buffer time between extension and grab
        robot.waitFor(275);

        // grab
        robot.grab(true);
        robot.waitWhile(() -> !robot.clawIntake.grabFinished());

        robot.setNextState(Robot.NextState.DONE);
        robot.waitWhile(() -> robot.getState() != Robot.RobotState.SAMPLE_READY);
    }

    public void chuckOut(){
        robot.goToPoint(new Pose2d(gx, cy, -Math.PI / 2), null, false, false, 0.95);
        Log.i("AUTO", "reached 1");
        // Outtake
        robot.setNextState(Robot.NextState.DONE);
        Log.i("AUTO", "reached 2");

        robot.waitWhile(() -> !robot.deposit.isOuttakeDone());
        Log.i("AUTO", "reached 3");

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