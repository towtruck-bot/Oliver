package org.firstinspires.ftc.teamcode.tests;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.utils.TelemetryUtil;

@TeleOp
@Config
public class MoveToTester extends LinearOpMode {
    public static double moveToX = 5.905314961, moveToY = 10.75;
    public static boolean set = true;

    public void runOpMode(){
        Robot robot = new Robot(hardwareMap);

        waitForStart();

        while(opModeIsActive()){
            if(set){
                robot.deposit.moveTo(moveToX, moveToY);
                set = !set;
            }

            TelemetryUtil.packet.put("Current X", robot.deposit.getCurrentX());
            // TelemetryUtil.packet.put("Current Y", robot.deposit.getCurrentY());

            robot.update();
        }
    }
}
