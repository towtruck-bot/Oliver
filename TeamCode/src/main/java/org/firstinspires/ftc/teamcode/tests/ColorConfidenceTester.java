package org.firstinspires.ftc.teamcode.tests;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Robot;

@TeleOp
@Config
public class ColorConfidenceTester extends LinearOpMode{
    public void runOpMode(){
        Robot robot = new Robot(hardwareMap);

        waitForStart();

        while(opModeIsActive()){
            robot.update();
        }
    }
}
