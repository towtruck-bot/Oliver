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

}
