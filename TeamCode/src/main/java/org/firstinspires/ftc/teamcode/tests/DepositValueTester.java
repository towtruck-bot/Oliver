package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.utils.Globals;
import org.firstinspires.ftc.teamcode.subsystems.deposit.Arm;

@TeleOp
public class DepositValueTester extends LinearOpMode {
    @Override
    public void runOpMode(){
        Robot robot = new Robot(hardwareMap);

        waitForStart();

        while(opModeIsActive()){
            telemetry.addData("baseHoriPosition: ", robot.deposit.arm.getHorizontalPos());
            telemetry.addData("baseVertPosition", robot.deposit.slides.getLength());
            telemetry.addData("armRotationServoValue: ", robot.deposit.arm.armRotation.getCurrentAngle());
            telemetry.addData("clawRotationServoValue: ", robot.deposit.arm.clawRotation.getCurrentAngle());
            telemetry.addData("clawGripServoValue: ", robot.deposit.arm.clawGrip.getCurrentAngle());
        }
    }
}
