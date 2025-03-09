package org.firstinspires.ftc.teamcode.tests;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.subsystems.drive.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.drive.OldDrivetrain;
import org.firstinspires.ftc.teamcode.utils.Pose2d;

@TeleOp
@Config
public class GoToPointTester extends LinearOpMode {
    public static double x = 0,  y = 0, h = 0;
    public static boolean finalAdjustment = false, slowDown = false, stop = false, goTo = false;

    @Override
    public void runOpMode() throws InterruptedException {
        Robot robot = new Robot(hardwareMap);
        OldDrivetrain drivetrain = robot.drivetrain;
        drivetrain.setPoseEstimate(new Pose2d(0,0, Math.toRadians(0)));

        waitForStart();

        while (!isStopRequested()) {
            if(goTo) {
                robot.drivetrain.goToPoint(new Pose2d(x, y, Math.toRadians(h)), finalAdjustment, stop, 1.0);
                robot.clawIntake.setIntakeTargetPos(0.0);
                goTo = !goTo;
            }
            robot.update();
        }
    }
}
