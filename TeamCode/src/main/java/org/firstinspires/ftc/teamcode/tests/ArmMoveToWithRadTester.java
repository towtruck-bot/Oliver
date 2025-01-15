package org.firstinspires.ftc.teamcode.tests;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Robot;

@TeleOp
@Config
public class ArmMoveToWithRadTester extends LinearOpMode {
    public static double armTargetRad = 0.0, targetY = 0, clawRotationRad = 0.0, clawGripRad = 0.0;

    public void runOpMode(){
        Robot robot = new Robot(hardwareMap);

        waitForStart();

        while(!isStopRequested()){
            robot.deposit.moveToWithRad(armTargetRad, targetY);
            robot.deposit.arm.clawRotation.setTargetAngle(clawRotationRad, 1.0);
            robot.deposit.arm.clawGrip.setTargetAngle(clawGripRad, 1.0);

            robot.update();
        }
    }
}
