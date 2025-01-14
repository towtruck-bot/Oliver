package org.firstinspires.ftc.teamcode.tests;

import static org.firstinspires.ftc.teamcode.utils.Globals.GET_LOOP_TIME;
import static org.firstinspires.ftc.teamcode.utils.Globals.START_LOOP;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.sensors.Sensors;
import org.firstinspires.ftc.teamcode.utils.TelemetryUtil;
import org.firstinspires.ftc.teamcode.utils.Utils;
import org.firstinspires.ftc.teamcode.utils.priority.PriorityMotor;

@TeleOp
public class SlideEncoderTester extends LinearOpMode{
    public void runOpMode(){
        Robot robot = new Robot(hardwareMap);
        Sensors sensors = robot.sensors;

        double motorPower = 0.0;

        waitForStart();

        while(!isStopRequested()){
            START_LOOP();
            robot.drivetrain.resetMinPowersToOvercomeFriction();

            if (gamepad1.b) {
                motorPower += 0.01;
            }

            if (gamepad1.x) {
                motorPower -= 0.01;
            }

            if (gamepad1.right_trigger > 0.1) {
                motorPower = 0;
            }

            motorPower = Utils.minMaxClip(motorPower, -1.0, 1.0);
            robot.deposit.slides.slidesMotors.setTargetPower(motorPower);

            robot.sensors.update();
            robot.hardwareQueue.update();

            telemetry.addData("Slides position", sensors.getSlidesPosition());
            telemetry.addData("motor power", motorPower);
            telemetry.update();

            TelemetryUtil.packet.put("Loop Time", GET_LOOP_TIME());
            TelemetryUtil.sendTelemetry();
        }
    }
}
