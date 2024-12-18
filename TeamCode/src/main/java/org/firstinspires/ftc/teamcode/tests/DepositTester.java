package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.utils.Globals;
import org.firstinspires.ftc.teamcode.utils.RunMode;

@TeleOp(group = "Test")
public class DepositTester extends LinearOpMode {
    public enum States{
        TRANSFER,

    }

    public void runOpMode(){
        Globals.RUNMODE = RunMode.TESTER;

        Robot robot = new Robot(hardwareMap);

        //now the goofy stuff starts
        while(opModeInInit()){
            robot.update();
        }

        while(!isStopRequested()){

        }
    }
}
