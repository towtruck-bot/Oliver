package org.firstinspires.ftc.teamcode.tests;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.subsystems.deposit.Arm;
import org.firstinspires.ftc.teamcode.subsystems.deposit.Deposit;

@TeleOp
@Config
public class CalculateMoveToTester extends LinearOpMode {
    public static double moveToX = Arm.armLengthT;
    public static double moveToY = 0.0;

    public static boolean set = false;
    public void runOpMode(){
        Robot robot = new Robot(hardwareMap);

        robot.deposit.state = Deposit.State.IDLE;

        waitForStart();

        while(opModeIsActive()){
            if(set){
                robot.deposit.targetX = moveToX;
                robot.deposit.targetY = moveToY;

                set = !set;
            }

            robot.deposit.calculateMoveTo();
            robot.deposit.updatePositions();

            telemetry.addData("horizontalPosition: ", robot.deposit.arm.getHorizontalPos());
            telemetry.addData("horizontalRail pos", robot.deposit.arm.getHorizontalPos());
            telemetry.addData("horizontal arm component", Math.cos(robot.deposit.arm.getArmRotation()) * robot.deposit.arm.armLength);
            telemetry.addData("verticalPosition: ", Math.sin(robot.deposit.arm.getArmRotation()) * robot.deposit.arm.armLength);
            telemetry.addData("slides height", robot.deposit.slides.getLength());
            telemetry.addData("vertical arm component", Math.sin(robot.deposit.arm.getArmRotation()) * robot.deposit.arm.armLength);

            robot.update();
        }
    }
}
