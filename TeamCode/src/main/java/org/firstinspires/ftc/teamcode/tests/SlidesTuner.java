package org.firstinspires.ftc.teamcode.tests;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.subsystems.deposit.Deposit;
import org.firstinspires.ftc.teamcode.subsystems.deposit.Slides;
import org.firstinspires.ftc.teamcode.utils.TelemetryUtil;

@TeleOp
@Config
public class SlidesTuner extends LinearOpMode {
    public static double targetSlidesHeight = 0.0;
    private static double largestVel = 0;
    public static boolean powerMode = false;
    public static double power = 0;

    public void runOpMode(){
        Robot robot = new Robot(hardwareMap);

        DcMotorEx motor = hardwareMap.get(DcMotorEx.class, "slidesMotor0");
        DcMotorEx motor1 = hardwareMap.get(DcMotorEx.class, "slidesMotor1");

        Slides slides = new Slides(robot);

        waitForStart();

        while(!isStopRequested()){
            if (!powerMode) {
                robot.deposit.holdSlides = true;
                robot.deposit.state = Deposit.State.TEST;
                robot.deposit.setDepositHeight(targetSlidesHeight);
            } else {
                robot.deposit.slides.slidesMotors.setPowerForced(power);
                if (Math.abs(robot.sensors.getSlidesVel()) > largestVel)
                    largestVel = Math.abs(robot.sensors.getSlidesVel());
                TelemetryUtil.packet.put("Slides vel", robot.sensors.getSlidesVel());
                TelemetryUtil.packet.put("Slides: largestVel", largestVel);
                TelemetryUtil.sendTelemetry();
                robot.sensors.update();
                continue;
            }

            TelemetryUtil.packet.put("Slides: Error", targetSlidesHeight - slides.getLength());
            TelemetryUtil.packet.put("Slides: Position", slides.getLength());
            TelemetryUtil.packet.put("Slides: Target Position", targetSlidesHeight);
            TelemetryUtil.packet.put("motor check0", motor.getCurrentPosition());
            TelemetryUtil.packet.put("motor check1", motor1.getCurrentPosition());

            robot.update();
        }
    }
}
