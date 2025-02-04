package org.firstinspires.ftc.teamcode.opmodes;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.subsystems.drive.Spline;
import org.firstinspires.ftc.teamcode.utils.Func;
import org.firstinspires.ftc.teamcode.utils.Globals;
import org.firstinspires.ftc.teamcode.utils.Pose2d;
import org.firstinspires.ftc.teamcode.utils.RunMode;
import org.firstinspires.ftc.teamcode.utils.Vector2;

@Autonomous(name = "BluePreloadBucketAuto")
public class BluePreloadBucketAuto extends LinearOpMode {
    private Robot robot;
    public void runOpMode(){
        doInitialization();

        Globals.autoStartTime = System.currentTimeMillis();

        moveToBelowBucket();
        score();
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
        robot.goToPoint(new Pose2d(48, 48, -Math.PI/2), () -> {
            return !isStopRequested();
        }, false, true, 0.8);

        //intake sample
        robot.setNextState(Robot.NextState.INTAKE_SAMPLE);
        robot.setIntakeExtension(13.0);
        robot.grab(true);

    }
}
