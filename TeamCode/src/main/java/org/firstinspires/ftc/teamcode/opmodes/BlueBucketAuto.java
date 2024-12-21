package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.utils.Globals;
import org.firstinspires.ftc.teamcode.utils.Pose2d;
import org.firstinspires.ftc.teamcode.utils.RunMode;

@Autonomous(name = "BlueBucketAuto")
public class BlueBucketAuto extends LinearOpMode {
    private Robot robot;


    public void runOpMode(){
        Globals.isRed = false;

        doInitialization();
        waitForStart();
        score1stPreload();
        score3Preloads();

    }

    public void doInitialization(){
        Globals.RUNMODE = RunMode.AUTO;

        robot = new Robot(hardwareMap);

        //robot.init(); TODO

        while (opModeInInit() && !isStopRequested()) {
            robot.drivetrain.setPoseEstimate(new Pose2d(66, -36, Math.PI)); // starting robot location for BlueBucketAuto todo: tune this starting position
            robot.update();
        }
    }
    public void score1stPreload(){
        // scoring the first sample
        robot.drivetrain.goToPoint(new Pose2d(48, -48, 315), true, true, 1.0); // go to basket
        // score basket

    }
    public void score3Preloads() {

        robot.drivetrain.goToPoint(new Pose2d(48, -48, 180), true, true, 1.0); // turn to preload
        // intake preload #1
        robot.drivetrain.goToPoint(new Pose2d(48, -48, 315), true, true, 1.0); // go to basket
        // score basket

        robot.drivetrain.goToPoint(new Pose2d(48, -48, 210), true, true, 1.0); // rotate to preload #2
        // intake preload #2
        robot.drivetrain.goToPoint(new Pose2d(48, -48, 315), true, true, 1.0); // rotate to basket
        // score basket

        robot.drivetrain.goToPoint(new Pose2d(48, -48, 225), true, true, 1.0); // rotate to preload #3 todo: maybe for preload #3 it's hard to intake since it's close to the wall
        // intake preload #3
        robot.drivetrain.goToPoint(new Pose2d(48, -48, 315), true, true, 1.0); // rotate to basket
        // score basket
    }

}
