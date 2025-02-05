package org.firstinspires.ftc.teamcode.opmodes;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.utils.Globals;
import org.firstinspires.ftc.teamcode.utils.Pose2d;
import org.firstinspires.ftc.teamcode.utils.RunMode;

@Autonomous(name = "BluePreloadBucketAuto")
public class BucketPreloadBlueAuto extends LinearOpMode {
    private Robot robot;

    public static boolean enableg1 = true, enableg2 = true, enableg3 = true, enabler = true;

    public static double g1x = 48, g1y = 48, g1h = -Math.PI/2, g1e = 13.0;
    public static double g2x = 54, g2y = 48, g2h = -Math.PI/2, g2e = 13.0;
    public static double g3x = 54, g3y = 48, g3h = -Math.PI/3, g3e = 15.75;
    public static double fx = 24.0, fy = 12.0, fh = Math.PI;

    public void runOpMode(){
        doInitialization();

        Globals.autoStartTime = System.currentTimeMillis();

        moveToBelowBucket();
        score();

        if(enableg1) {
            get1stGround();
            moveToBelowBucket();
            score();
        }

        if(enableg2) {
            get2ndGround();
            moveToBelowBucket();
            score();
        }

        if(enableg3) {
            get3rdGround();
            moveToBelowBucket();
            score();
        }

        if(enabler) {
            goToTeleOpStart();
        }
    }

    public void doInitialization(){
        Globals.isRed = false;
        Globals.RUNMODE = RunMode.AUTO;
        Globals.hasSamplePreload = true;

        robot = new Robot(hardwareMap);

        robot.sensors.resetPosAndIMU();

        while(opModeInInit() && !isStopRequested()){
            robot.sensors.setOdometryPosition(48.0 - Globals.TRACK_WIDTH / 2.0, 72.0 - Globals.TRACK_LENGTH / 2.0, Math.PI/2);
            robot.deposit.setDepositHeight(0.0);

            robot.update();
        }

        // Do vision stuff here?
    }

    public void moveToBelowBucket(){
        //robot current state, SAMPLE_READY
        robot.goToPoint(new Pose2d(55, 55, 5 * Math.PI/4), () -> {
            return !isStopRequested();
        }, false, true, 0.8);

        //raise slides
        robot.updateDepositHeights(false, true);
        Log.i("why is this not working", String.valueOf(robot.deposit.getDepositHeight()));
        robot.setNextState(Robot.NextState.DEPOSIT);

        while (!robot.deposit.isSampleHigh())
            robot.update();
    }

    public void score(){
        //robot current state, DEPOSIT_BUCKET
        robot.goToPoint(new Pose2d(58, 58, 5 * Math.PI/4), () -> {
            return !isStopRequested();
        }, true, true, 0.8);

        //release sample
        robot.setNextState(Robot.NextState.DONE);

        robot.goToPoint(new Pose2d(55, 55, 5 * Math.PI/4), () -> {
            return !isStopRequested();
        }, false, true, 0.8);

        // This was the incredibly cooked arm part
        while (!robot.deposit.isRetractDone() || true){
            robot.update();
        }
    }

    public void get1stGround(){
        robot.goToPoint(new Pose2d(g1x, g1y, g1h), () -> {
            return !isStopRequested();
        }, false, true, 0.8);

        //intake sample
        robot.setNextState(Robot.NextState.INTAKE_SAMPLE);
        robot.setIntakeExtension(g1e);

        // this will cause clawIntake FSM to lower and grab. previous length was manually measured such that the claw would flip down on the sample
        robot.grab(true);
        robot.setNextState(Robot.NextState.DONE);
    }

    public void get2ndGround(){
        robot.goToPoint(new Pose2d(g2x, g2y, g2h), () -> {
            return !isStopRequested();
        }, false, true, 0.8);

        //intake sample
        robot.setNextState(Robot.NextState.INTAKE_SAMPLE);
        robot.setIntakeExtension(g2e);

        // this will cause clawIntake FSM to lower and grab. previous length was manually measured such that the claw would flip down on the sample
        robot.grab(true);
        robot.setNextState(Robot.NextState.DONE);
    }

    public void get3rdGround(){
        robot.goToPoint(new Pose2d(g3x, g3y, g3h), () -> {
            return !isStopRequested();
        }, false, true, 0.8);

        //intake sample
        robot.setNextState(Robot.NextState.INTAKE_SAMPLE);
        robot.setIntakeExtension(g3e);

        // this will cause clawIntake FSM to lower and grab. previous length was manually measured such that the claw would flip down on the sample
        robot.grab(true);
        robot.setNextState(Robot.NextState.DONE);
    }

    public void goToTeleOpStart(){
        robot.goToPoint(new Pose2d(fx, fy, fh), () -> {
            return !isStopRequested();
        }, true, true, 0.8);
    }
}
