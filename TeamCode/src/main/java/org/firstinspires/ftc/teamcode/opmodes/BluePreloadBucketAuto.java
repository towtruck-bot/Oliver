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

        // TODO: Reset encoder values
    }

    public void scorePreload(){
        // go to in front of bucket, angle bot to directly face bucket
        robot.drivetrain.goToPoint(new Pose2d(1.504653589141163,-0.47072564848254705, 0.781970739364624), false, true, 1.0);
        double startTime = System.currentTimeMillis();
        while (robot.drivetrain.isBusy() || System.currentTimeMillis() > startTime + 5000) {
            robot.update();
        }

        // start deposit from hold, enter sample sequence, reach sample raise
        robot.setNextState(Robot.NextState.DEPOSIT);
        while (!robot.deposit.slides.inPosition(0.9) ){
            robot.update();
        }

        // move closer to bucket
        robot.drivetrain.goToPoint(new Pose2d(1.662282774028439,-0.3064954456019752,0.7775459289550781), false, true, 1.0);
        startTime = System.currentTimeMillis();
        while (robot.drivetrain.isBusy() || System.currentTimeMillis() > startTime + 5000) {
            robot.update();
        }

        // deposit
        robot.setNextState(Robot.NextState.DEPOSIT);

        // back up
        robot.drivetrain.goToPoint(new Pose2d(1.504653589141163, -0.47072564848254705, 3.097757339477539), false, false, 1.0);
        startTime = System.currentTimeMillis();
        while (robot.drivetrain.isBusy() || System.currentTimeMillis() > startTime + 5000) {
            robot.update();
        }

        // go to observation zone
        robot.drivetrain.goToPoint(new Pose2d(-3.105215890607563, 0.06, 3.097757339477539), false, true, 1.0);
        startTime = System.currentTimeMillis();
        while (robot.drivetrain.isBusy() || System.currentTimeMillis() > startTime + 5000) {
            robot.update();
        }
    }
}
