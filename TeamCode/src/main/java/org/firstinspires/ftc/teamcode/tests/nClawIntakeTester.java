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
    public static nClawIntake.State stateDashboard = nClawIntake.State.READY;
    public static boolean setState = false;
    public static boolean perserveState = false;
    public static double targetX = 0;
    public static double targetY = 0;
    public static double targetHeading = 0;
    public static nClawIntake.State state = nClawIntake.State.READY;
    public static double targetLength = 0;
    public static boolean useCamera = true;
    public static boolean grab = false;
    public static boolean setGrab = false;
    public static boolean crankThatSoulaBoy = false;
    public static boolean setSample = false;
    public static boolean confirmTransfer;
    private double k = 0;

    public void runOpMode(){
        Globals.RUNMODE = RunMode.AUTO;
        Robot robot = new Robot(hardwareMap);

        waitForStart();
        //robot.deposit.retract();

        while(!isStopRequested()){
            if(setState) {
                robot.nclawIntake.state = stateDashboard;
                setState = !setState;
            } else if (perserveState)
                robot.nclawIntake.state = state;

            if (setSample) {
                robot.nclawIntake.confirmGrab();
                setSample = !setSample;
            }
            if (confirmTransfer) {
                robot.nclawIntake.finishTransfer();
                confirmTransfer = !confirmTransfer;
            }
            if (setGrab) {
                robot.nclawIntake.grab(grab);
                setGrab = !setGrab;
            }

            robot.nclawIntake.setIntakeLength(targetLength);
            robot.nclawIntake.useCamera(useCamera);


            if (!crankThatSoulaBoy)
                robot.nclawIntake.setTargetPose(new Pose2d(targetX, targetY, targetHeading));
            else
                robot.nclawIntake.setTargetPose(new Pose2d(Math.cos(k) * 5 + 20, Math.sin(k) * 5, 0));
            TelemetryUtil.packet.put("current state", robot.nclawIntake.state);

            robot.update();
            k += Math.PI * 2 / (1000.0 / 20);
        }
    }
}
