package org.firstinspires.ftc.teamcode.tests;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.subsystems.intake.Intake;
import org.firstinspires.ftc.teamcode.utils.Globals;
import org.firstinspires.ftc.teamcode.utils.RunMode;
import org.firstinspires.ftc.teamcode.utils.TelemetryUtil;

@TeleOp
@Config
public class JamesFunnyIntakeTester extends LinearOpMode {
    public static Intake.IntakeState stateDashboard = Intake.IntakeState.IDLE;
    public static Intake.IntakeRollerState stateRollerDashboard = Intake.IntakeRollerState.OFF;
    public static boolean set = false;
    public static double targetLength = 0;
//    public static double extensionControlTargetPositionDash = Intake.extendedMinPosition;

    public void runOpMode(){
        Robot robot = new Robot(hardwareMap);
        Globals.RUNMODE = RunMode.TELEOP;

        waitForStart();

        while(!isStopRequested()){
            if(set){
//                robot.intake.extensionControlTargetPosition = extensionControlTargetPositionDash;
                robot.intake.intakeState = stateDashboard;
                set = !set;
            }

            robot.intake.extTargetLen = targetLength;

            robot.intake.intakeRollerState = stateRollerDashboard;

            TelemetryUtil.packet.put("current intake state", robot.intake.intakeState);

            robot.update();
        }
    }
}
