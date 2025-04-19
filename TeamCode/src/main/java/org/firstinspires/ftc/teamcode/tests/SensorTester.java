package org.firstinspires.ftc.teamcode.tests;

import com.acmerobotics.dashboard.config.Config;
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
@Config
public class SensorTester extends LinearOpMode {
    public static boolean intakeLight = true;

    @Override
    public void runOpMode() throws InterruptedException {
        Globals.RUNMODE = RunMode.TESTER;
        Globals.TESTING_DISABLE_CONTROL = true;

        Robot robot = new Robot(hardwareMap);
        Sensors sensors = robot.sensors;

        robot.sensors.setOdometryPosition(48.0 - Globals.ROBOT_REVERSE_LENGTH, 72.0 - Globals.ROBOT_WIDTH / 2, Math.PI);
        robot.update();

        waitForStart();

        while (!isStopRequested()) {
            robot.sensors.update();
            robot.vision.update();

//            telemetry.addData("leftOdo", sensors.getOdometry()[0]);
//            telemetry.addData("rightOdo", sensors.getOdometry()[1]);
//            telemetry.addData("backOdo", sensors.getOdometry()[2]);

            Pose2d pos = sensors.getOdometryPosition();
            String data = String.format(Locale.US, "{X: %.3f, Y: %.3f, H: %.3f}", pos.getX(), pos.getY(), pos.getHeading());

            robot.nclawIntake.intakeLight.setState(intakeLight);

            if (!robot.vision.isConnected()) {
                gamepad1.rumble(100);
                gamepad2.rumble(100);
            }

            telemetry.addData("Position", data);
            telemetry.addData("Slides position", sensors.getSlidesPos());
            telemetry.addData("extendo slides position", sensors.getExtendoPos());
            telemetry.addData("Limelight connection", robot.vision.isConnected() ? "everything is fine" : "freaking packet yo");
            TelemetryUtil.packet.put("PS Value", robot.nclawIntake.readPS());

            telemetry.update();
            TelemetryUtil.sendTelemetry();
        }
    }
}
