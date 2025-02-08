package org.firstinspires.ftc.teamcode.tests;

import android.util.Log;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.utils.Func;
import org.firstinspires.ftc.teamcode.utils.Globals;
import org.firstinspires.ftc.teamcode.utils.Pose2d;
import org.firstinspires.ftc.teamcode.utils.RunMode;
import org.firstinspires.ftc.teamcode.utils.TelemetryUtil;

@TeleOp
@Config
public class AccuracyTester extends LinearOpMode {
    public static boolean goTo = false, finalAdjustment = false, stop = false;
    public static double x = 48.0 - Globals.TRACK_WIDTH / 2.0, y = 72.0 - Globals.TRACK_LENGTH / 2.0, heading = Math.PI/2;

    public void runOpMode(){
        Globals.RUNMODE = RunMode.TESTER;
        Globals.TESTING_DISABLE_CONTROL = true;

        Robot robot = new Robot(hardwareMap);
        robot.sensors.setOdometryPosition(x, y, heading);
//        Pose2d currPos = robot.sensors.getOdometryPosition();
//        Log.i("what the fuck is happening", currPos.x + " " + currPos.y + " " + currPos.heading);

        waitForStart();

        while(opModeInInit()){
            robot.update();
        }

        waitForStart();

        while(!isStopRequested()){
            Pose2d estimate = robot.sensors.getOdometryPosition();

            if(goTo){
                robot.drivetrain.goToPoint(new Pose2d(x, y, heading), finalAdjustment, stop, 0.8);
                goTo = !goTo;
            }

            TelemetryUtil.packet.put("Pinpoint:: x", estimate.getX());
            TelemetryUtil.packet.put("Pinpoint:: y", estimate.getY());
            TelemetryUtil.packet.put("Pinpoint:: heading", estimate.getHeading());

            robot.update();
        }
    }
}
