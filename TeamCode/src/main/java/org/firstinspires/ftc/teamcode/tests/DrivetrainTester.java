package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.subsystems.drive.Drivetrain;
import org.firstinspires.ftc.teamcode.utils.Globals;
import org.firstinspires.ftc.teamcode.utils.Pose2d;
import org.firstinspires.ftc.teamcode.utils.RunMode;

@TeleOp
public class DrivetrainTester extends LinearOpMode {
    private Robot robot;

    public void runOpMode(){
        doInitialization();

        moveCycle();
        robot.drivetrain.state = Drivetrain.State.IDLE;

        while(!isStopRequested()){
            robot.update();
        }
    }

    public void doInitialization(){
        Globals.isRed = false;
        Globals.RUNMODE = RunMode.AUTO;
        Globals.hasSamplePreload = false;

        robot = new Robot(hardwareMap);
        robot.setAbortChecker(() -> !isStopRequested());

        robot.sensors.resetPosAndIMU();

        while (opModeInInit() && !isStopRequested()) {
            robot.sensors.setOdometryPosition(48.0, 48, Math.PI);
            robot.deposit.setDepositHeight(0.0);

            robot.update();
        }
    }

    public void moveCycle(){
        robot.goToPoint(new Pose2d(48.0, 48.0, Math.PI), null, false, false, 0.8);

        for(int i = 0; i < 5; i++){
//            robot.followSpline(
//                    new Spline(
//                        new Pose2d(24.0, 60.0, Math.PI),
//                        4
//                    )
//                        .addPoint(new Pose2d(0.0, 60.0, Math.PI))
//                        .addPoint(new Pose2d(-24.0, 48.0, -Math.PI))
//                        .addPoint(new Pose2d(-48.0, 24.0, -Math.PI)),
//                    this::opModeIsActive
//            );
            robot.goToPoint(new Pose2d(24.0, 60.0, Math.PI), null, false, false, 0.8);
            robot.goToPoint(new Pose2d(0.0, 60.0, Math.PI), null, false, false, 0.8);
            robot.goToPoint(new Pose2d(-24.0, 48.0, -Math.PI), null, false, false, 0.8);
            robot.goToPoint(new Pose2d(-48.0, 24.0, -Math.PI), null, false, true, 0.8);

//            robot.followSpline(
//                    new Spline(
//                        new Pose2d(-24.0, 48.0, Math.PI),
//                            4
//                    )
//                        .addPoint(new Pose2d(0.0, 60.0, Math.PI))
//                        .addPoint(new Pose2d(24.0, 60.0, Math.PI))
//                        .addPoint(new Pose2d(48.0, 48.0, Math.PI)),
//                    this::opModeIsActive
//            );

            robot.goToPoint(new Pose2d(-24.0, 48.0, -Math.PI), null, false, false, 0.8);
            robot.goToPoint(new Pose2d(0.0, 60.0, Math.PI), null, false, false, 0.8);
            robot.goToPoint(new Pose2d(24.0, 60.0, Math.PI), null, false, false, 0.8);
            robot.goToPoint(new Pose2d(48.0, 48.0, Math.PI), null, false, true, 0.8);
        }

        robot.goToPoint(new Pose2d(48.0, 48.0, Math.PI), null, true, true, 0.8);
    }
}
