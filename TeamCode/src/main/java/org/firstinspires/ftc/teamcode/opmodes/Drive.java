package org.firstinspires.ftc.teamcode.opmodes;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.utils.ButtonToggle;
import org.firstinspires.ftc.teamcode.utils.Globals;
import org.firstinspires.ftc.teamcode.utils.RunMode;

@TeleOp
public class Drive extends LinearOpMode {
    @Override
    public void runOpMode() {
        Globals.RUNMODE = RunMode.TELEOP;

        Robot robot = new Robot(hardwareMap);
        Servo armL = hardwareMap.get(Servo.class, "armServoL");
        Servo armR = hardwareMap.get(Servo.class, "armServoR");
        Servo claw = hardwareMap.get(Servo.class, "claw");
        Servo diffR = hardwareMap.get(Servo.class, "diffyL");
        Servo diffL = hardwareMap.get(Servo.class, "diffyR");

        telemetry.addData("State", "READY TO START");
        telemetry.update();

        while (opModeInInit()) {
            robot.update();
        }
        double armPos = 0.5;
        double rotate = 0;
        ButtonToggle g1t = new ButtonToggle();
        boolean clawOpen = false;
        while (!isStopRequested()) {
            robot.drivetrain.drive(gamepad1);
            armL.setPosition(armPos);
            armR.setPosition(1-armPos);


            diffR.setPosition(armPos + rotate);
            diffL.setPosition(1-armPos - rotate);

            if (armPos < 0.2) {
                armPos = 0.2;
            }
            boolean speedy = false;


            speedy = gamepad1.left_trigger > 0.3;

            if (gamepad1.a) {
                if (speedy) {
                    armPos += 0.003;
                }
                else {
                    armPos += 0.002;
                }
            }
            if (gamepad1.y) {
                if (speedy){
                    armPos -= 0.003;
                }
                else {
                    armPos-= 0.002;
                }
            }

            if (gamepad1.x)
                rotate += 0.001;
            if (gamepad1.b)
                rotate -= 0.001;
            if (g1t.isClicked(gamepad1.right_bumper)) {
                Log.e("press", "e");
                if (clawOpen) {
                    claw.setPosition(1.412*0.2162104887);
                }
                else {
                    claw.setPosition(0.407*0.2162104887);
                }
                clawOpen = !clawOpen;
            }

            telemetry.addData("armPos", armPos);

            robot.update();

            telemetry.update();
        }

    }
}
