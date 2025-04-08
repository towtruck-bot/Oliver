package org.firstinspires.ftc.teamcode.tests;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.subsystems.intake.nClawIntake;
import org.firstinspires.ftc.teamcode.utils.Globals;
import org.firstinspires.ftc.teamcode.utils.TelemetryUtil;

@TeleOp
@Config
public class ExtendoTuner extends LinearOpMode {
    public static double targetLength = 0.0;

    public void runOpMode(){
        Robot robot = new Robot(hardwareMap);
        robot.nclawIntake.state = nClawIntake.State.TEST;

        Globals.TESTING_DISABLE_CONTROL = false;

        waitForStart();

        while(!isStopRequested()){
            robot.nclawIntake.setExtendoTargetPos(targetLength);

            TelemetryUtil.packet.put("Extendo Target", robot.nclawIntake.getExtendoTargetPos());
            TelemetryUtil.packet.put("Extendo Current Pos", robot.sensors.getExtendoPos());

            robot.update();
        }
    }
}
