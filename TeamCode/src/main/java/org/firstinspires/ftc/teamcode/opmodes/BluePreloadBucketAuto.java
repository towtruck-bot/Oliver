package org.firstinspires.ftc.teamcode.opmodes;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.subsystems.drive.Spline;
import org.firstinspires.ftc.teamcode.utils.Func;
import org.firstinspires.ftc.teamcode.utils.Globals;
import org.firstinspires.ftc.teamcode.utils.Pose2d;
import org.firstinspires.ftc.teamcode.utils.RunMode;
import org.firstinspires.ftc.teamcode.utils.Vector2;

@Autonomous(name = "BluePreloadBucketAuto")
public class BluePreloadBucketAuto extends LinearOpMode {
    private Robot robot;
    public void runOpMode(){
        Globals.isRed = false;

        doInitialization();
        waitForStart();
        Globals.autoStartTime = System.currentTimeMillis();

        moveToBelowBucket();
    }

    public void doInitialization(){
        Globals.RUNMODE = RunMode.AUTO;
        Globals.hasSamplePreload = true;

        robot = new Robot(hardwareMap);

        robot.sensors.resetPosAndIMU();

        while(opModeInInit() && !isStopRequested()){
            robot.sensors.setOdometryPosition(24.0 - Globals.TRACK_WIDTH/2, 72.0 - Globals.TRACK_LENGTH/2, -Math.PI / 2);
            robot.deposit.setDepositHeight(0.0);

            robot.update();
        }

        // Do vision stuff here?
    }

    public void moveToBelowBucket(){
        robot.goToPoint(new Pose2d(58, 58, Math.PI/4), (Func) this, false, true, 0.8);
        robot.setNextState(Robot.NextState.DEPOSIT);
    }
}
