package org.firstinspires.ftc.teamcode.tests;

import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.utils.Globals;
import org.firstinspires.ftc.teamcode.utils.Pose2d;
import org.firstinspires.ftc.teamcode.utils.RunMode;
import org.firstinspires.ftc.teamcode.utils.TelemetryUtil;
import org.firstinspires.ftc.teamcode.utils.Vector2;

@TeleOp
@Config
public class IntakeWithLL extends LinearOpMode {
    public static double extensionLength = 10;
    public static boolean restart = false;
    public static boolean grab = false;
    public static boolean setGrab = false;
    public static boolean confirm = false;
    public static boolean finish = false;
    public static boolean extend = false;
    private boolean grabbing = false;

    @Override
    public void runOpMode() throws InterruptedException {
        Globals.RUNMODE = RunMode.AUTO;
        Robot robot = new Robot(hardwareMap);

        waitForStart();
        robot.nclawIntake.resetExtendoEncoders();

        robot.nclawIntake.useCamera(true);
        robot.nclawIntake.extend();
        robot.vision.startDetection();
        while (opModeIsActive()) {
            robot.vision.setOffset(new Vector2(robot.nclawIntake.getIntakeRelativeToRobot(), 0));
            Pose2d blockPos = robot.vision.getBlockPos();

            TelemetryUtil.packet.put("blockPos.x", blockPos.x);
            TelemetryUtil.packet.put("blockPos.y", blockPos.y);
            TelemetryUtil.packet.put("blockPos.heading", blockPos.heading);

            Canvas c = TelemetryUtil.packet.fieldOverlay();
            c.setStroke("#444400");
            c.strokeCircle(blockPos.x, blockPos.y, 3);
            c.strokeLine(blockPos.x, blockPos.y, blockPos.x + 5 * Math.sin(blockPos.heading), blockPos.y + 5 * Math.cos(blockPos.heading));

            if (restart) {
                robot.vision.startDetection();
                restart = false;
            }

            if (robot.vision.concurrentDetections() > 20 && !grabbing) {
                robot.nclawIntake.setTargetPose(robot.vision.getBlockPos());
                robot.nclawIntake.grab(true);
                grabbing = true;
            }

            if (confirm) {
                robot.nclawIntake.confirmGrab();
                confirm = false;
            }

            if (finish) {
                robot.nclawIntake.finishTransfer();
                finish = false;
            }

            if (extend) {
                robot.nclawIntake.extend();
                extend = false;
                grabbing = false;
                robot.vision.resetConcurrentDetections();
            }

            robot.nclawIntake.setIntakeLength(extensionLength);

            robot.update();
        }
    }
}
