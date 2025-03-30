package org.firstinspires.ftc.teamcode.tests.tuners;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.subsystems.intake.ClawIntake;
import org.firstinspires.ftc.teamcode.utils.Globals;
import org.firstinspires.ftc.teamcode.utils.TelemetryUtil;

@TeleOp
@Config
@Disabled
public class IntakePIDTuner extends LinearOpMode {
    public static double targetExtension = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        Globals.TESTING_DISABLE_CONTROL = false;
        Robot robot = new Robot(hardwareMap);

        robot.clawIntake.clawIntakeState = ClawIntake.ClawIntakeState.TEST;

        waitForStart();

        while (opModeIsActive()) {
            robot.clawIntake.setIntakeTargetPos(targetExtension);

            // Logging for PID graphs
            TelemetryUtil.packet.put("Intake Target", robot.clawIntake.getIntakeTargetPos());
            TelemetryUtil.packet.put("Intake Current Position", robot.sensors.getExtendoPos());

            robot.update();
        }


    }
}
