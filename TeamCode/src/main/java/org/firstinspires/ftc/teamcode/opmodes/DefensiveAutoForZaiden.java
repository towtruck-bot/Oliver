package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.subsystems.deposit.nDeposit;
import org.firstinspires.ftc.teamcode.subsystems.intake.nClawIntake;
import org.firstinspires.ftc.teamcode.utils.Globals;
import org.firstinspires.ftc.teamcode.utils.LogUtil;
import org.firstinspires.ftc.teamcode.utils.Pose2d;
import org.firstinspires.ftc.teamcode.utils.RunMode;

@Config
@Disabled
@Autonomous(name = "B. Defense Sample Auto")
public class DefensiveAutoForZaiden extends LinearOpMode {
    private Robot robot;

    public static double sox = 24, soy = 12;
    public static double px = 24, py = -12;
    public static double sx = 24, sy = 12;
    public static double st1 = Math.toRadians(180), st2 = Math.toRadians(210);

    public void runOpMode() {
        Globals.RUNMODE = RunMode.AUTO;
        Globals.isRed = false;
        Globals.hasSamplePreload = false;
        Globals.hasSpecimenPreload = false;

        robot = new Robot(hardwareMap);
        robot.setStopChecker(() -> !isStopRequested());

        robot.sensors.resetPosAndIMU();

        robot.ndeposit.state = nDeposit.State.HOLD;
        robot.nclawIntake.setGrabMethod(nClawIntake.GrabMethod.MANUAL_AIM);
        robot.nclawIntake.setTargetType(nClawIntake.Target.MANUAL);
        robot.nclawIntake.setRetryGrab(true);

        robot.ndeposit.presetDepositHeight(false, true, false);
        robot.nclawIntake.disableRestrictedHoldPos();
        robot.nclawIntake.setAutoEnableCamera(false);

        robot.drivetrain.setBrakePad(false);

        while(opModeInInit() && !isStopRequested()) {
            robot.sensors.setOdometryPosition(46.5 - Globals.ROBOT_REVERSE_LENGTH, 72.0 - Globals.ROBOT_WIDTH / 2, Math.PI);
            //robot.sensors.setOdometryPosition(-36 + Globals.ROBOT_WIDTH, 70.5 - Globals.ROBOT_REVERSE_LENGTH, -Math.PI / 2);
            robot.update();
        }

        //if (!isStopRequested()) LogUtil.init();
        //LogUtil.drivePositionReset = true;
        robot.update();

        robot.drivetrain.goToPoint(
                new Pose2d(sox, soy, Math.PI),
                false,
                false,
                1.0
        );
        robot.waitWhile(() -> robot.drivetrain.isBusy());

        robot.drivetrain.goToPoint(
                new Pose2d(px, py, Math.PI),
                false,
                true,
                1.0
        );
        robot.nclawIntake.extend();
        robot.nclawIntake.setGrab(true);
        robot.waitWhile(() -> robot.drivetrain.isBusy());

        while (!isStopRequested()) {
            robot.drivetrain.goToPoint(
                    new Pose2d(sx, sy, st2),
                    false,
                    false,
                    1.0
            );
            robot.waitWhile(() -> robot.drivetrain.isBusy());

            robot.drivetrain.goToPoint(
                    new Pose2d(sx, sy, st1),
                    false,
                    false,
                    1.0
            );
            robot.waitWhile(() -> robot.drivetrain.isBusy());
        }
    }
}
