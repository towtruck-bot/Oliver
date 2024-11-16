package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.utils.Globals;
import org.firstinspires.ftc.teamcode.utils.RunMode;
import org.firstinspires.ftc.teamcode.vision.Vision;

public class ParkAutoBlue extends LinearOpMode {
    private Robot robot;

    public void runOpMode(){
        Globals.isRed = false;

        doInitialization();
        waitForStart();

        double startTime = System.nanoTime();
        while(opModeIsActive()){
            //if 10 seconds
            if(System.nanoTime() - startTime < Math.pow(10, 10)){
                robot.drivetrain.leftFront.setTargetPower(0.2);
                robot.drivetrain.leftRear.setTargetPower(0.2);
                robot.drivetrain.rightRear.setTargetPower(0.2);
                robot.drivetrain.rightFront.setTargetPower(0.2);
            }else{
                robot.drivetrain.leftFront.setTargetPower(0.0);
                robot.drivetrain.leftRear.setTargetPower(0.0);
                robot.drivetrain.rightRear.setTargetPower(0.0);
                robot.drivetrain.rightFront.setTargetPower(0.0);
            }
        }
    }

    public void doInitialization(){
        Globals.RUNMODE = RunMode.AUTO;

        robot = new Robot(hardwareMap);

    }

}
