package org.firstinspires.ftc.teamcode.tests;

import static org.firstinspires.ftc.teamcode.utils.Globals.GET_LOOP_TIME;
import static org.firstinspires.ftc.teamcode.utils.Globals.START_LOOP;

import android.util.Log;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.utils.ButtonToggle;
import org.firstinspires.ftc.teamcode.utils.Globals;
import org.firstinspires.ftc.teamcode.utils.RunMode;
import org.firstinspires.ftc.teamcode.utils.TelemetryUtil;
import org.firstinspires.ftc.teamcode.utils.Utils;
import org.firstinspires.ftc.teamcode.utils.priority.HardwareQueue;
import org.firstinspires.ftc.teamcode.utils.priority.PriorityDevice;
import org.firstinspires.ftc.teamcode.utils.priority.PriorityServo;
import org.firstinspires.ftc.teamcode.utils.priority.PriorityServoAxonEnc;
import org.firstinspires.ftc.teamcode.utils.priority.nPriorityServo;

import java.util.ArrayList;

@Config
@TeleOp(group = "Test")
public class ServoTester extends LinearOpMode {

    public static double intakePower = 1.0;
    public static boolean usePosition = false;
    public static double position = Math.toRadians(60);

    @Override
    public void runOpMode() throws InterruptedException {
        Globals.RUNMODE = RunMode.TESTER;
        Globals.TESTING_DISABLE_CONTROL = true;

        Robot robot = new Robot(hardwareMap);
        nPriorityServo tester = new nPriorityServo(
                new Servo[] {hardwareMap.get(Servo.class, "testServo")},
                "tester",
                nPriorityServo.ServoType.HITEC,
                0,
                1,
                0,
                new boolean[] {false},
                2, 5);
        robot.hardwareQueue.addDevice(tester);

        HardwareQueue hardwareQueue = robot.hardwareQueue;

        ArrayList<nPriorityServo> servos = new ArrayList<>();

        ButtonToggle buttonY = new ButtonToggle();
        ButtonToggle buttonA = new ButtonToggle();
        ButtonToggle left_bumper = new ButtonToggle();

        int servoSize = 0;
        boolean intakeOn = false;

        // getting number of servos we have;
        for (PriorityDevice device : hardwareQueue.devices) {
            if (device instanceof nPriorityServo) {
                servos.add((nPriorityServo) device);
                servoSize++;
            }
        }

        double[] servoPos = new double[servoSize];
        for (int i = 0; i < servoSize; i ++){
            servoPos[i] = servos.get(i).basePos;
        }

        int servoIndex = 0;
        double numLoops = 0;
        double totalTime = 0;

        TelemetryUtil.setup();

        waitForStart();
        while (!isStopRequested()) {
            START_LOOP();
            hardwareQueue.update();
            robot.sensors.update();

            numLoops ++;

            if (gamepad1.x) {
                servoPos[servoIndex] += 0.001;
            }
            if (gamepad1.b){
                servoPos[servoIndex] -= 0.001;
            }

            //TODO: Should the min-max clip not correspond to the Servo's own min max? - James
            servoPos[servoIndex] = Utils.minMaxClip(servoPos[servoIndex], 0.0, 1.0);

            // figuring out time to set servo pos
            long start = System.nanoTime();
            servos.get(servoIndex).setTargetPos(usePosition ? position : servoPos[servoIndex], 1.0);
            double elapsedTime = (System.nanoTime()-start)/1000000000.0;
            totalTime += elapsedTime;

            // incrementing / decrementing servoIndex
            if (buttonY.isClicked(gamepad1.y)) {
                servoIndex += 1;
            }

            if (buttonA.isClicked(gamepad1.a)) {
                servoIndex -= 1;
            }

            // if the servoIndex exceeds servoSize wrap around
            servoIndex = (servoIndex + servoSize) % servoSize;
            telemetry.addData("if the servo is reversed, the pos will be reversed too", "lol");
            telemetry.addData("servoName", servos.get(servoIndex).name);
            telemetry.addData("servoIndex", servoIndex);
            telemetry.addData("servoPos", servoPos[servoIndex]);
            telemetry.addData("servoAngle", servos.get(servoIndex).getCurrentAngle());
            telemetry.addData("averageServoTime", totalTime/numLoops);
            //telemetry.addData("v4Encoder", v4Bar);
            telemetry.addData("angle", servos.get(servoIndex).getCurrentAngle());
            telemetry.addData("targetAngle", servos.get(servoIndex).getTargetAngle());
            /*if (servos.get(servoIndex) instanceof PriorityServoAxonEnc) {
                telemetry.addData("voltage", " " + ((PriorityServoAxonEnc) servos.get(servoIndex)).getEncoderVoltage());
                telemetry.addData("angle", " " + ((PriorityServoAxonEnc) servos.get(servoIndex)).getEncoderAngle());
            }*/

            TelemetryUtil.packet.put("Loop Time", GET_LOOP_TIME());
            TelemetryUtil.sendTelemetry();
            telemetry.update();
        }
    }
}
