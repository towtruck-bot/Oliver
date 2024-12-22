package org.firstinspires.ftc.teamcode.tests.tuners;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.subsystems.intake.Intake;
import org.firstinspires.ftc.teamcode.utils.TelemetryUtil;

@Disabled
@TeleOp
@Config
public class IntakePIDTuner extends LinearOpMode {
    public static double targetExtension = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        Robot robot = new Robot(hardwareMap);
/*
        robot.intake.intakeState = Intake.IntakeState.TESTER;
        robot.intake.setRollerOff();

        waitForStart();

        while (opModeIsActive()) {
            robot.intake.extTargetLen = targetExtension;

            // Logging for PID graphs
            TelemetryUtil.packet.put("Intake Target", robot.intake.extTargetLen);
            TelemetryUtil.packet.put("Intake Current Position", robot.intake.extCurrentLen);

            robot.update();
        }

 */
    }
}
