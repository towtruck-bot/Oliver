package org.firstinspires.ftc.teamcode.opmodes;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.utils.Globals;
import org.firstinspires.ftc.teamcode.utils.Pose2d;
import org.firstinspires.ftc.teamcode.utils.RunMode;
import org.firstinspires.ftc.teamcode.utils.Vector2;

@Autonomous(name = "BluePreloadBucketAuto")
public class BluePreloadBucketAuto extends LinearOpMode {
    private Robot robot;
    public void runOpMode(){
        Globals.isRed = false;

        doInitialization();
        waitForStart();

        scorePreload();
    }

    public void doInitialization(){
        Globals.RUNMODE = RunMode.AUTO;
        Globals.hasSamplePreload = true;

        robot = new Robot(hardwareMap);


    }

    public void scorePreload(){
        //go to in front of bucket, angle bot to directly face bucket


        robot.drivetrain.goToPoint(new Pose2d(48,-48, Math.toRadians(315)), true, true, 1.0);
        double startTime = System.currentTimeMillis();
        while (robot.drivetrain.isBusy() || System.currentTimeMillis() > startTime + 5000) {
            robot.update();
        }

        //start deposit from hold, enter sample sequence, reach sample raise
        robot.setNextState(Robot.NextState.DEPOSIT);
        while (!robot.deposit.slides.inPosition(0.9) ){
            robot.update();
        }

        robot.setNextState(Robot.NextState.DEPOSIT);

        robot.drivetrain.goToPoint(new Pose2d(48, -48, Math.toRadians(-90)), false, false, 1.0);
        startTime = System.currentTimeMillis();
        while (robot.drivetrain.isBusy() || System.currentTimeMillis() > startTime + 5000) {
            robot.update();
        }
        robot.drivetrain.goToPoint(new Pose2d(60, 60, Math.toRadians(-90)), true, true, 1.0);
        startTime = System.currentTimeMillis();
        while (robot.drivetrain.isBusy() || System.currentTimeMillis() > startTime + 5000) {
            robot.update();
        }


        // calculate if claw location is above bucket, if not, adjust

        // release then retract

        // move to park zone to get specimens/ready to sweep three blue samples
    }
}
