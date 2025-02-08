package org.firstinspires.ftc.teamcode.opmodes;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.utils.Globals;
import org.firstinspires.ftc.teamcode.utils.RunMode;
import org.firstinspires.ftc.teamcode.utils.Vector2;
import org.firstinspires.ftc.teamcode.vision.Vision;

@Autonomous(name = "ParkAuto", preselectTeleOp = "A. Teleop")
public class ParkAutoBlue extends LinearOpMode {
    private Robot robot;

    public void runOpMode(){
        Globals.isRed = false;
        Globals.RUNMODE = RunMode.AUTO;
        Globals.hasSamplePreload = true;

        robot = new Robot(hardwareMap);

        while (opModeInInit()) {
            robot.update();
        }

        double startTime = System.currentTimeMillis();
        Log.e("Here", "here1");

        while(!isStopRequested() && System.currentTimeMillis() - startTime < 3000) {
            Log.e("Here", "IN LOOP");
            //if 10 seconds
            robot.drivetrain.setMoveVector(new Vector2(0.2, 0), 0);
            robot.update();
        }
        Log.e("Here", "here2");
        robot.drivetrain.setMoveVector(new Vector2(), 0);
        robot.update();
    }
}
