package org.firstinspires.ftc.teamcode.tests;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.subsystems.intake.nClawIntake;
import org.firstinspires.ftc.teamcode.utils.Globals;
import org.firstinspires.ftc.teamcode.utils.RunMode;
import org.firstinspires.ftc.teamcode.utils.TelemetryUtil;
import org.firstinspires.ftc.teamcode.utils.Pose2d;

@TeleOp(group = "Test")
@Config
public class nClawIntakeTester extends LinearOpMode {
    public static nClawIntake.nClawIntakeState stateDashboard = nClawIntake.nClawIntakeState.READY;
    public static boolean set = false;
    public static boolean perserveState = true;
    public static double targetX = 0;
    public static double targetY = 0;
    public static double targetHeading = 0;
    public static nClawIntake.nClawIntakeState state = nClawIntake.nClawIntakeState.READY;
    public static double targetLength = 0;
    public static boolean useCamera = true;

    public void runOpMode(){
        Globals.RUNMODE = RunMode.AUTO;
        Robot robot = new Robot(hardwareMap);

        waitForStart();
        robot.deposit.retract();

        while(!isStopRequested()){
            if(set) {
                robot.nclawIntake.clawIntakeState = stateDashboard;
                set = !set;
            } else if (perserveState)
                robot.nclawIntake.clawIntakeState = state;

            robot.nclawIntake.setIntakeLength(targetLength);
            robot.nclawIntake.setCamera(useCamera);
            robot.nclawIntake.setTargetPose(new Pose2d(targetX, targetY, targetHeading));
            TelemetryUtil.packet.put("current state", robot.nclawIntake.clawIntakeState);

            robot.update();
        }
    }
}
