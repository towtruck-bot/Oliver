package org.firstinspires.ftc.teamcode.tests;

import static org.firstinspires.ftc.teamcode.utils.Globals.START_LOOP;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.utils.ButtonToggle;
import org.firstinspires.ftc.teamcode.utils.TelemetryUtil;
import org.firstinspires.ftc.teamcode.utils.Utils;
import org.firstinspires.ftc.teamcode.utils.priority.PriorityServo;

import java.util.ArrayList;

@TeleOp
public class CoordinateGetter extends LinearOpMode {
    public void runOpMode(){
        Robot robot = new Robot(hardwareMap);

        ArrayList<PriorityServo> servos = new ArrayList<>();

        ButtonToggle buttonY = new ButtonToggle();
        ButtonToggle buttonA = new ButtonToggle();

        servos.add(robot.deposit.arm.horizontalRail);
        servos.add(robot.deposit.arm.armRotation);

        int servoIndex = 0;
        int servoSize = servos.size();

        double[] servoPos = new double[servoSize];
        for (int i = 0; i < servoSize; i ++){
            servoPos[i] = servos.get(i).basePos;
        }
        TelemetryUtil.setup();

        waitForStart();

        while (!isStopRequested()) {
            START_LOOP();
            robot.hardwareQueue.update();
            robot.sensors.update();

            if (gamepad1.x) {
                servoPos[servoIndex] += 0.001;
            }
            if (gamepad1.b){
                servoPos[servoIndex] -= 0.001;
            }

            if(gamepad1.left_bumper){
                robot.deposit.arm.clawRotation.setTargetAngle(1.0499, 1.0);
            }

            servoPos[servoIndex] = Utils.minMaxClip(servoPos[servoIndex], servos.get(servoIndex).minPos, servos.get(servoIndex).maxPos);


            servos.get(servoIndex).setTargetPose(servoPos[servoIndex], 1.0);

            // incrementing / decrementing servoIndex
            if (buttonY.isClicked(gamepad1.y)) {
                servoIndex += 1;
            }

            if (buttonA.isClicked(gamepad1.a)) {
                servoIndex -= 1;
            }

            // if the servoIndex exceeds servoSize wrap around
            servoIndex = (servoIndex + servoSize) % servoSize;

            double horizontalPosition = robot.deposit.arm.getHorizontalPos() + Math.cos(robot.deposit.arm.getArmRotation()) * robot.deposit.arm.armLength;
            double verticalPosition = Math.sin(robot.deposit.arm.getArmRotation()) * robot.deposit.arm.armLength;

            telemetry.addData("servoName", servos.get(servoIndex).name);
            telemetry.addData("servoPos", servoPos[servoIndex]);
            telemetry.addData("servoAngle", servos.get(servoIndex).getCurrentAngle());
            telemetry.addData("horizontalPosition: ", horizontalPosition);
            telemetry.addData("horizontalRail pos", robot.deposit.arm.getHorizontalPos());
            telemetry.addData("horizontal arm component", Math.cos(robot.deposit.arm.getArmRotation()) * robot.deposit.arm.armLength);
            telemetry.addData("verticalPosition: ", verticalPosition);

            TelemetryUtil.sendTelemetry();
            telemetry.update();
        }
    }
}
