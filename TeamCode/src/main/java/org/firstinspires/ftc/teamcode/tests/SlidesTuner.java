package org.firstinspires.ftc.teamcode.tests;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.subsystems.deposit.Slides;
import org.firstinspires.ftc.teamcode.utils.TelemetryUtil;

@TeleOp
@Config
public class SlidesTuner extends LinearOpMode {
    public static double targetSlidesHeight = 0.0;

    public void runOpMode(){
        Robot robot = new Robot(hardwareMap);

        Slides slides = new Slides(robot);

        waitForStart();

        while(!isStopRequested()){
            slides.setTargetLength(targetSlidesHeight);

            TelemetryUtil.packet.put("Slides: Error", targetSlidesHeight - slides.getLength());
            TelemetryUtil.packet.put("Slides: Position", slides.getLength());
            TelemetryUtil.packet.put("Slides: Target Position", targetSlidesHeight);

            slides.update();
            robot.update();
        }
    }
}
