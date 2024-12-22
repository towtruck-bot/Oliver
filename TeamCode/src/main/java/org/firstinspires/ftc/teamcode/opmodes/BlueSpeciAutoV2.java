package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.utils.Globals;
import org.firstinspires.ftc.teamcode.utils.Pose2d;
import org.firstinspires.ftc.teamcode.utils.RunMode;

@Autonomous(name = "BlueSpeciAutoV2")
public class BlueSpeciAutoV2 extends LinearOpMode {
    private Robot robot;

    public void runOpMode(){
        Globals.isRed = false;

        doInitialization();
        waitForStart();

        firstSpecimen();
        clipsToHP();


    }

    public void doInitialization(){
        Globals.RUNMODE = RunMode.AUTO;

        robot = new Robot(hardwareMap);

        //robot.init(); TODO

        while (opModeInInit() && !isStopRequested()) {
            robot.drivetrain.setPoseEstimate(new Pose2d(66, 0, Math.PI)); // starting robot location for bluespeciauto todo: tune this starting position
            robot.update();
        }
    }
    public void firstSpecimen() {
        // raise slides
        robot.drivetrain.goToPoint(new Pose2d(24, 0, 180), true, true, 1.0); // go forward towards the submersible todo: tune this (will it ram into the submersible?)
        // speci deposit
        robot.drivetrain.goToPoint(new Pose2d(48, 0, 180), false, true, 1.0); // backup todo: too far?
    }
    public void clipsToHP() {
        // three cycles to move 3 game element to human player

        robot.drivetrain.goToPoint(new Pose2d(48, 24, 135), true, true, 1.0); // go towards preload #1 todo: there is probably a more effective way to rotate the bot via code

    }
}
