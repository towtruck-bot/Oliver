package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.utils.TelemetryUtil;
import org.firstinspires.ftc.teamcode.utils.priority.PriorityMotor;

@TeleOp
public class SlideEncoderTester extends LinearOpMode{
    public void runOpMode(){
        Robot robot = new Robot(hardwareMap);

        PriorityMotor testMotor = new PriorityMotor(
                robot.hardwareMap.get(DcMotorEx.class, "testMotor"),
                "testMotor",
                4, 5, robot.sensors
        );
        robot.hardwareQueue.addDevice(testMotor);

        waitForStart();

        while(!isStopRequested()){

            if(gamepad1.x){
                testMotor.power += 0.01;
            }

            if(gamepad1.b){
                testMotor.power -= 0.01;
            }

            TelemetryUtil.packet.put("Test Motor Pos", testMotor.motor[0].getCurrentPosition());
            robot.update();
        }
    }
}
