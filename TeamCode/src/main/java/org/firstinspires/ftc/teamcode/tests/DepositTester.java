package org.firstinspires.ftc.teamcode.tests;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.subsystems.deposit.Deposit;
import org.firstinspires.ftc.teamcode.utils.TelemetryUtil;

@TeleOp(group = "Test")
@Config
public class DepositTester extends LinearOpMode {
    public static Deposit.State stateDashboard = Deposit.State.RETRACT;
    public static boolean set = true;

    public void runOpMode(){
        /*Robot robot = new Robot(hardwareMap);

        waitForStart();

        while(!isStopRequested()){
            if(set){
                robot.deposit.state = stateDashboard;
                set = !set;
            }

            TelemetryUtil.packet.put("current state", robot.deposit.state);

            robot.update();
        }*/
    }
}
