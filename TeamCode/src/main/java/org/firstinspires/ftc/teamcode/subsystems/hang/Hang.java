package org.firstinspires.ftc.teamcode.subsystems.hang;


import com.qualcomm.robotcore.hardware.CRServo;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.utils.priority.PriorityCRServo;

public class Hang {
    private final PriorityCRServo leftServo;
    private final PriorityCRServo rightServo;
    private final PriorityCRServo l3Servo;

    public Hang(Robot robot) {
        CRServo leftHang = robot.hardwareMap.get(CRServo.class, "leftHang");
        CRServo rightHang = robot.hardwareMap.get(CRServo.class, "rightHang");
        CRServo l3Hang = robot.hardwareMap.get(CRServo.class, "l3Hang");

        leftServo = new PriorityCRServo(new CRServo[]{leftHang}, "leftHang", 1, 2);
        rightServo = new PriorityCRServo(new CRServo[]{rightHang}, "rightHang", 1, 2);
        l3Servo = new PriorityCRServo(new CRServo[]{l3Hang}, "l3Hang", 1, 2);

        robot.hardwareQueue.addDevice(leftServo);
        robot.hardwareQueue.addDevice(rightServo);
        robot.hardwareQueue.addDevice(l3Servo);
    }

    public void update() {}

    public void quickTurnOnOff() {
        onFORCED();
        offFORCED();
    }

    public void onFORCED() {
        leftServo.servo[0].setPower(0.1);
        rightServo.servo[0].setPower(0.1);
    }

    public void offFORCED() {
        leftServo.servo[0].setPower(0.0);
        rightServo.servo[0].setPower(0.0);
    }

    public void on() {
        leftServo.setTargetPower(1.0);
        rightServo.setTargetPower(1.0);
    }

    public void reverse() {
        leftServo.setTargetPower(-1.0);
        rightServo.setTargetPower(-1.0);
    }

    public void off() {
        leftServo.setTargetPower(0.0);
        rightServo.setTargetPower(0.0);
    }

    public void leftReverse() { leftServo.setTargetPower(-1.0); }
    public void rightReverse() { rightServo.setTargetPower(-1.0); }
    public void l3Pull() { l3Servo.setTargetPower(-1.0); }
    public void leftUp() { leftServo.setTargetPower(1.0); }
    public void rightUp() { rightServo.setTargetPower(1.0); }
    public void l3Up() { l3Servo.setTargetPower(1.0); }
    public void leftOff() { leftServo.setTargetPower(0.0); }
    public void rightOff() { rightServo.setTargetPower(0.0); }
    public void l3Off() { l3Servo.setTargetPower(0.0); }
}
