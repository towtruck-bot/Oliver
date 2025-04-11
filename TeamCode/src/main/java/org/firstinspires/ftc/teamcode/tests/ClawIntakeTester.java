package org.firstinspires.ftc.teamcode.tests;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.subsystems.deposit.Deposit;
import org.firstinspires.ftc.teamcode.subsystems.intake.ClawIntake;
import org.firstinspires.ftc.teamcode.utils.TelemetryUtil;

//@TeleOp(group = "Test")
//@Config
public class ClawIntakeTester extends LinearOpMode {
    public static ClawIntake.ClawIntakeState stateDashboard = ClawIntake.ClawIntakeState.RETRACT;
    public static boolean set = true;

    public void runOpMode(){
        /*Robot robot = new Robot(hardwareMap);

        waitForStart();

        robot.deposit.prepareTransfer();

        while(!isStopRequested()){
            if(set){
                robot.clawIntake.clawIntakeState = stateDashboard;
                set = !set;
            }

            TelemetryUtil.packet.put("current state", robot.deposit.state);

            robot.update();
        }*/
    }
}
