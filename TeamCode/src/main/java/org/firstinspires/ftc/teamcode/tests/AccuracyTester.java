package org.firstinspires.ftc.teamcode.tests;

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
    public static boolean goTo = false;
    public static double x = 0.0, y = 0.0, heading = 0.0;

    public void runOpMode(){
        Globals.RUNMODE = RunMode.TESTER;
        Globals.TESTING_DISABLE_CONTROL = true;

        Robot robot = new Robot(hardwareMap);

        waitForStart();

        //now at (0, 0, 0)
        robot.sensors.resetPosAndIMU();
        robot.sensors.setOdometryPosition(0.0, 0.0, 0.0);

        while(opModeInInit()){
            robot.update();
        }

        waitForStart();

//        for(int i = 0; i < 5; i++){
//            robot.drivetrain.goToPoint(new Pose2d(84.0, 12.0, 0.0), false, false, 0.8);
//            robot.drivetrain.goToPoint(new Pose2d(96, 24.0, Math.PI/2), false, false, 0.8);
//            robot.drivetrain.goToPoint(new Pose2d(84.0, 12.0, 0.0), false, false, 0.8);
//            robot.drivetrain.goToPoint(new Pose2d(12.0, 12.0, 0.0), false, false, 0.8);
//        }
//
//        robot.drivetrain.goToPoint(new Pose2d(0.0, 0.0, 0.0), true, true, 0.8);

        boolean arrived = false;
        while(!isStopRequested()){
            Pose2d estimate = robot.sensors.getOdometryPosition();

            robot.drivetrain.goToPoint(new Pose2d(x, y, Math.toRadians(heading)), true, true, 0.8);


            //robot.drivetrain.goToPoint(new Pose2d(40, 0.0, 0.0), false, false, 0.8);
//            robot.goToPoint(new Pose2d(0.0, 0.0, 0.0), (Func) this, false, false, 0.8);


            TelemetryUtil.packet.put("Pinpoint:: x", estimate.getX());
            TelemetryUtil.packet.put("Pinpoint:: y", estimate.getY());
            TelemetryUtil.packet.put("Pinpoint:: heading", estimate.getHeading());

            robot.update();
        }
    }
}
