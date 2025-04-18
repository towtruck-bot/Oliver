package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.utils.Globals;
import org.firstinspires.ftc.teamcode.utils.RunMode;

public class DefensiveAutoForZaiden extends LinearOpMode {
    private Robot robot;

    public void runOpMode(){
        Globals.RUNMODE = RunMode.AUTO;
        Globals.isRed = false;
        Globals.hasSamplePreload = false;
        Globals.hasSpecimenPreload = false;

        robot = new Robot(hardwareMap);
        robot.setStopChecker(() -> !isStopRequested());

        robot.sensors.resetPosAndIMU();


    }
}
