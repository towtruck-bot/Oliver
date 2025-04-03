package org.firstinspires.ftc.teamcode.tests;

import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.subsystems.deposit.nDeposit;
import org.firstinspires.ftc.teamcode.subsystems.intake.nClawIntake;
import org.firstinspires.ftc.teamcode.utils.AngleUtil;
import org.firstinspires.ftc.teamcode.utils.ButtonToggle;
import org.firstinspires.ftc.teamcode.utils.Globals;
import org.firstinspires.ftc.teamcode.utils.Pose2d;
import org.firstinspires.ftc.teamcode.utils.RunMode;
import org.firstinspires.ftc.teamcode.utils.TelemetryUtil;
import org.firstinspires.ftc.teamcode.utils.Vector2;

@TeleOp(name = "B. IntakeWithLL")
@Config
public class IntakeWithLL extends LinearOpMode {
    public static boolean autoGrab = false;
    public static boolean useKnown = false;
    public static double keepX = 5;
    public static double keepY = 3;

    @Override
    public void runOpMode() throws InterruptedException {
        Globals.RUNMODE = RunMode.AUTO;
        Robot robot = new Robot(hardwareMap);

        robot.drivetrain.setPoseEstimate(new Pose2d(0, 0, 0));

        waitForStart();

        robot.nclawIntake.useCamera(true);
        //robot.nclawIntake.extend();
        robot.drivetrain.setPoseEstimate(new Pose2d(0, 0, 0));

        ButtonToggle x1 = new ButtonToggle();
        ButtonToggle y1 = new ButtonToggle();
        ButtonToggle a1 = new ButtonToggle();
        ButtonToggle b1 = new ButtonToggle();

        while (opModeIsActive()) {
            robot.vision.setOffset(robot.nclawIntake.getIntakeRelativeToRobot());
            Pose2d blockPos = robot.vision.getBlockPos();

            TelemetryUtil.packet.put("blockPos.x", blockPos.x);
            TelemetryUtil.packet.put("blockPos.y", blockPos.y);
            TelemetryUtil.packet.put("blockPos.heading", blockPos.heading);
            RobotLog.e("blockPos.heading " + Math.toDegrees(blockPos.heading));

            Canvas c = TelemetryUtil.packet.fieldOverlay();
            c.setStroke("#444400");
            c.strokeCircle(blockPos.x, blockPos.y, 3);
            c.strokeLine(blockPos.x, blockPos.y, blockPos.x + 5 * Math.sin(blockPos.heading), blockPos.y + 5 * Math.cos(blockPos.heading));

            if (y1.isClicked(gamepad1.y)) robot.nclawIntake.extend();

            if (x1.isClicked(gamepad1.x)) robot.ndeposit.startSampleDeposit();

            if (a1.isClicked(gamepad1.a)) robot.ndeposit.deposit();

            if (b1.isClicked(gamepad1.b)) robot.nclawIntake.retract();

            double intakeControl1 = robot.drivetrain.smoothControls(-gamepad1.right_stick_y);
            //robot.nclawIntake.setIntakeLength(robot.nclawIntake.getIntakeTargetPos() + 0.4 * intakeControl1);
            if (!useKnown) {
                robot.nclawIntake.removeKnown();
                robot.nclawIntake.setIntakeLength(10);
            } else {
                robot.nclawIntake.setKnownIntakePose(new Pose2d(keepX, keepY, 0));
            }

            robot.nclawIntake.setAutoGrab(autoGrab);

            /*if (setExtend) {
                robot.vision.startDetection();
                robot.nclawIntake.extend();
                setExtend = false;
            }

            if (setRetract) {
                robot.vision.stopDetection();
                robot.nclawIntake.retract();
                setRetract = false;
            }

            if (transferRequest) {
                robot.ndeposit.startTransfer();
                transferRequest = false;
            }*/

            if (robot.nclawIntake.isTransferReady() && robot.ndeposit.isTransferReady()) {
                robot.nclawIntake.finishTransfer();
                robot.ndeposit.finishTransfer();
            }

            /*if (setGrab) {
                robot.nclawIntake.setTargetPose(new Pose2d(
                        blockPos.x,
                        blockPos.y,
                        AngleUtil.mirroredClipAngleTolerence(-blockPos.heading, Math.toRadians(20))
                ));
                robot.nclawIntake.grab(grab);
                setGrab = false;
            }*/

            TelemetryUtil.packet.put("velocityLowPass", robot.vision.getVelocityLowPass());
            //robot.nclawIntake.setIntakeLength(extensionLength);
            robot.drivetrain.drive(gamepad1);

            robot.update();
        }
    }
}
