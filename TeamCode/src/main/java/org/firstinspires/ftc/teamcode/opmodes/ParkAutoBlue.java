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


    }

    public void doInitialization(){
        Globals.RUNMODE = RunMode.AUTO;

        robot = new Robot(hardwareMap);


    }
}
