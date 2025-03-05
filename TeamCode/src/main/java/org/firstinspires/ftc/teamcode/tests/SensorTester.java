package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.sensors.Sensors;
import org.firstinspires.ftc.teamcode.utils.Globals;
import org.firstinspires.ftc.teamcode.utils.Pose2d;
import org.firstinspires.ftc.teamcode.utils.RunMode;
import org.firstinspires.ftc.teamcode.utils.TelemetryUtil;

import java.util.Locale;

@TeleOp(group = "Test")
public class SensorTester extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Globals.RUNMODE = RunMode.TESTER;
        Globals.TESTING_DISABLE_CONTROL = true;

        Robot robot = new Robot(hardwareMap);
        Sensors sensors = robot.sensors;

        waitForStart();

        while (!isStopRequested()) {
            robot.sensors.update();

//            telemetry.addData("leftOdo", sensors.getOdometry()[0]);
//            telemetry.addData("rightOdo", sensors.getOdometry()[1]);
//            telemetry.addData("backOdo", sensors.getOdometry()[2]);

            Pose2d pos = sensors.getOdometryPosition();
            String data = String.format(Locale.US, "{X: %.3f, Y: %.3f, H: %.3f}", pos.getX(), pos.getY(), pos.getHeading());
            telemetry.addData("Position", data);

            telemetry.addData("Slides position", sensors.getSlidesPos());
            telemetry.addData("extendo slides position", sensors.getExtendoPos());

            telemetry.update();
            TelemetryUtil.sendTelemetry();
        }
    }
}
