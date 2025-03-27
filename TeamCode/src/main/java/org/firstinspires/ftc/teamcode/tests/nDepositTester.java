package org.firstinspires.ftc.teamcode.tests;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.subsystems.deposit.nDeposit;
import org.firstinspires.ftc.teamcode.utils.Globals;
import org.firstinspires.ftc.teamcode.utils.RunMode;
import org.firstinspires.ftc.teamcode.utils.TelemetryUtil;

@TeleOp
@Config
public class nDepositTester extends LinearOpMode {
    public static nDeposit.State state = nDeposit.State.IDLE;

    @Override
    public void runOpMode() throws InterruptedException {
        Globals.RUNMODE = RunMode.AUTO;
        Robot robot = new Robot(hardwareMap);

        waitForStart();
        //robot.deposit.retract();

        while(!isStopRequested()){
            robot.ndeposit.state = state;

            TelemetryUtil.packet.put("current state", robot.ndeposit.state);

            robot.update();
        }
    }
}
