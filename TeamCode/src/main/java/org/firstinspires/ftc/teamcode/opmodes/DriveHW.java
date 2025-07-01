package org.firstinspires.ftc.teamcode.opmodes;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import static java.lang.Math.abs;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.sensors.Sensors;
import org.firstinspires.ftc.teamcode.utils.priority.PriorityMotor;
import org.firstinspires.ftc.teamcode.utils.priority.nPriorityServo;

public class DriveHW extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {

        Robot robot = new Robot(hardwareMap);
        Sensors sensors = new Sensors(robot);
        HardwareMap hardwareMap = null;
        PriorityMotor leftFront = new PriorityMotor(hardwareMap.get(DcMotorEx.class, "leftFront"), "leftFront", 3, 5, sensors);
        PriorityMotor leftBack = new PriorityMotor(hardwareMap.get(DcMotorEx.class, "leftBack"), "leftBack", 3, 5, sensors);
        PriorityMotor rightBack = new PriorityMotor(hardwareMap.get(DcMotorEx.class, "rightBack"), "rightBack", 3, 5, sensors);
        PriorityMotor rightFront = new PriorityMotor(hardwareMap.get(DcMotorEx.class, "rightFront"), "rightFront", 3, 5, sensors);
        PriorityMotor turret = new PriorityMotor(hardwareMap.get(DcMotorEx.class, "turret"), "turret", 2, 4, sensors);
        PriorityMotor intake = new PriorityMotor(hardwareMap.get(DcMotorEx.class, "intake"), "intake", 1, 1, sensors);
        PriorityMotor vertSlideRight = new PriorityMotor(hardwareMap.get(DcMotorEx.class, "vertSlideRight"), "vertSlideRight", 2, 4, sensors);
        PriorityMotor vertSlideLeft = new PriorityMotor(hardwareMap.get(DcMotorEx.class, "vertSlideLeft"), "vertSlideLeft", 2, 4, sensors);
        nPriorityServo claw = new nPriorityServo(new Servo[] {robot.hardwareMap.get(Servo.class, "claw"), robot.hardwareMap.get(Servo.class, "claw")}, "claw", nPriorityServo.ServoType.AXON_MINI, 0, 0, 0, new boolean[] {false, true}, 3.0, 5.0); //idk min, max, or base so i left them as 0
        nPriorityServo clawRotate = new nPriorityServo(new Servo[] {robot.hardwareMap.get(Servo.class, "clawRotate"), robot.hardwareMap.get(Servo.class, "clawRotate")}, "clawRotate", nPriorityServo.ServoType.AXON_MINI, 0, 0, 0, new boolean[] {false, true}, 3.0, 5.0);
        nPriorityServo intakeLeft = new nPriorityServo(new Servo[] {robot.hardwareMap.get(Servo.class, "intakeLeft"), robot.hardwareMap.get(Servo.class, "intakeLeft")}, "intakeLeft", nPriorityServo.ServoType.AXON_MINI, 0, 0, 0, new boolean[] {false, true}, 1.5, 3.0);
        nPriorityServo intakeRight = new nPriorityServo(new Servo[] {robot.hardwareMap.get(Servo.class, "intakeRight"), robot.hardwareMap.get(Servo.class, "intakeRight")}, "intakeRight", nPriorityServo.ServoType.AXON_MINI, 0, 0, 0, new boolean[] {false, true}, 1.5, 3.0);
        nPriorityServo horizontalExtendLeft = new nPriorityServo(new Servo[] {robot.hardwareMap.get(Servo.class, "horizontalExtendLeft"), robot.hardwareMap.get(Servo.class, "horizontalExtendLeft")}, "horizontalExtendLeft", nPriorityServo.ServoType.AXON_MINI, 0, 0, 0, new boolean[] {false, true}, 1.0, 1.5);
        nPriorityServo horizontalExtendRight = new nPriorityServo(new Servo[] {robot.hardwareMap.get(Servo.class, "horizontalExtendRight"), robot.hardwareMap.get(Servo.class, "horizontalExtendRight")}, "horizontalExtendRight", nPriorityServo.ServoType.AXON_MINI, 0, 0, 0, new boolean[] {false, true}, 1.0, 1.5);

        robot.hardwareQueue.addDevice(leftFront);
        robot.hardwareQueue.addDevice(leftBack);
        robot.hardwareQueue.addDevice(rightFront);
        robot.hardwareQueue.addDevice(rightBack);
        robot.hardwareQueue.addDevice(turret);
        robot.hardwareQueue.addDevice(intake);
        robot.hardwareQueue.addDevice(vertSlideLeft);
        robot.hardwareQueue.addDevice(vertSlideRight);
        robot.hardwareQueue.addDevice(claw);
        robot.hardwareQueue.addDevice(clawRotate);
        robot.hardwareQueue.addDevice(intakeLeft);
        robot.hardwareQueue.addDevice(intakeRight);
        robot.hardwareQueue.addDevice(horizontalExtendLeft);
        robot.hardwareQueue.addDevice(horizontalExtendRight);


//Drive Motors - 4
//Turret Rotation - 1
//Active Intake - 1
//Vertical Slides - 2
//Servos:
//Claw Actuation (Open/close, rotate) - 2
//Horizontal Extension (2 for v4bar,  2 for extension) - 4

        while (opModeIsActive()){
            double y = -gamepad1.left_stick_y; // Remember, Y stick is reversed!
            double x = gamepad1.left_stick_x;
            double turn = gamepad1.right_stick_x;

            double LF = y + x + turn;
            double LR = y - x + turn;
            double RF = y - x - turn;
            double RR = y + x - turn;
            if ((abs(y) + abs(x) + abs(turn))>1){
                LF = LF/(abs(y) + abs(x) + abs(turn));
                LR = LR/(abs(y) + abs(x) + abs(turn));
                RF = RF/(abs(y) + abs(x) + abs(turn));
                RF = RF/(abs(y) + abs(x) + abs(turn));
            }

            leftFront.setTargetPower(LF);
            leftBack.setTargetPower(LR);
            rightFront.setTargetPower(RF);
            rightBack.setTargetPower(RR);

        }
    }
}
