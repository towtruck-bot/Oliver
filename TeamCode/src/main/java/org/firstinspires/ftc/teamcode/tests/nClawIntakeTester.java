package org.firstinspires.ftc.teamcode.tests;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.subsystems.deposit.Deposit;
import org.firstinspires.ftc.teamcode.subsystems.intake.ClawIntake;
import org.firstinspires.ftc.teamcode.subsystems.intake.nClawIntake;
import org.firstinspires.ftc.teamcode.utils.TelemetryUtil;
import org.firstinspires.ftc.teamcode.utils.Pose2d;

@TeleOp(group = "Test")
@Config
public class nClawIntakeTester extends LinearOpMode {
    public static nClawIntake.nClawIntakeState stateDashboard = nClawIntake.nClawIntakeState.READY;
    public static boolean set = true;
    public static Pose2d target;

    public void runOpMode(){
        Robot robot = new Robot(hardwareMap);

        waitForStart();
        robot.deposit.retract();

        while(!isStopRequested()){
            if(set){
                robot.nclawIntake.clawIntakeState = stateDashboard;
                robot.nclawIntake.setTargetPose(target);
                set = !set;
            }

            TelemetryUtil.packet.put("current state", robot.deposit.state);

            robot.update();
        }
    }
}
