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

    public static boolean enablePreload = true, enableGround = true, enableScore = true;

    public static double gx1 = -49.0, gy1 = 25.75;
    public static double gx2 = -60.0, gy2 = 25.75;
    public static double gx3 = -70.0, gy3 = 25.75;
    public static double ox = -70, oy = -70;

    public static double setupX = -36.5, setupY = 63.9;
    public static double baseScoreX = -5.9, baseScoreY = 41.45, scoreY = 28.75;
    public static double[] targetX = new double[] {0.0, 2.5, 5.0, 7.5, 10.0};

    public void runOpMode(){
        doInitialization();

        Globals.autoStartTime = System.currentTimeMillis();

        if(enablePreload){
            score(-Globals.TRACK_WIDTH / 2.0);
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
        robot.waitWhile(() -> !robot.deposit.readyToRam());
        // ^Wait for fully raised slides and arm at correct angle^

        // Ram forward
        robot.goToPoint(new Pose2d(offset, scoreY, Math.PI/2), null, false, false, false, 0.85);
        robot.waitFor(75);

        // Release + backup
        robot.setNextState(Robot.NextState.DONE);
        robot.goToPoint(new Pose2d(baseScoreX, baseScoreY, Math.PI/2), null, false, false, false, 0.95);
    }

    public void move3Ground(){
        intakeGroundAt(gx1, gy1);
        outtake();

        intakeGroundAt(gx2, gy2);
        outtake();

        intakeGroundAt(gx3, gy3);
        outtake();
    }

    public void intakeGroundAt(double gx, double gy){
        robot.goToPoint(new Pose2d(gx, gy), null, true, true, true, 0.95);

        robot.setNextState(Robot.NextState.INTAKE_SAMPLE);
        robot.setIntakeExtension(robot.drivetrain.getExtension());

        robot.waitWhile(() -> !robot.intake.isExtended());

        robot.waitFor(50);

        robot.grab(true);
        robot.waitWhile(() -> !robot.intake.grabFinished());

        robot.setNextState(Robot.NextState.DONE);
    }

    public void outtake(){
        robot.goToPoint(new Pose2d(ox, oy), null, true, true, true, 0.95);

        robot.setNextState(Robot.NextState.INTAKE_SAMPLE);
        robot.setIntakeExtension(robot.drivetrain.getExtension());

        robot.waitWhile(() -> !robot.intake.isExtended());

        robot.waitFor(50);

        robot.grab(false);

        robot.setNextState(Robot.NextState.DONE);
    }

    public void grabAndSetUp(double offset){
        // Pre-position
        robot.goToPoint(new Pose2d(setupX, setupY - 6.0, Math.PI / 2.0), null, false, false, false, 0.95);

        // Prepare to grab
        robot.setNextState(Robot.NextState.GRAB_SPECIMEN);
        robot.goToPoint(new Pose2d(setupX, setupY, Math.PI / 2.0), null, false, true, true, 0.8);

        // Grab
        robot.setNextState(Robot.NextState.GRAB_SPECIMEN);
        robot.waitWhile(() -> !robot.deposit.isSpecimenReady());

        // Move to setup
        // Offset is used such that the specimen wont clip on each other
        robot.goToPoint(new Pose2d(offset, baseScoreY, Math.PI / 2.0), null, false, false, false, 0.95);
    }

    public void park(){
        robot.goToPoint(new Pose2d(setupX - 6.0, setupY, Math.PI / 2.0), null, false, false, true, 1.0);
    }
}
