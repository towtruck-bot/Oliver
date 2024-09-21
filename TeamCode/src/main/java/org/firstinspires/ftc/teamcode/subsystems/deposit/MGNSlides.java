package org.firstinspires.ftc.teamcode.subsystems.deposit;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.utils.priority.PriorityCRServo;
import org.firstinspires.ftc.teamcode.utils.priority.PriorityServo;

public class MGNSlides {
    public Robot robot;
    public PriorityServo armAngle, armPosition;
    public double currPos;
    public final double ropeLength = 1.0;
    public final double servoRadius = 0.1;
    public MGNSlides(Robot robot) {
        this.robot = robot;

        armAngle = new PriorityServo(robot.hardwareMap.get(Servo.class, "armAngle"), "armAngle", PriorityServo.ServoType.SPEED, 1.0, 1.0, 1.0, 5.0, false, 3.0, 5.0);
        armPosition = new PriorityServo(robot.hardwareMap.get(Servo.class, "armPosition"), "armPosition", PriorityServo.ServoType.SPEED,1.0,1.0,1.0,5.0,false,3.0,5.0);
        currPos = 0.0;
    }

    public void moveMGN(double newPos){
        double diff = newPos - currPos;
        armPosition.setTargetAngle(diff/(2*Math.PI), 1.0); //target angle wrong, fig out how to adjust - James
        currPos = newPos;
    }

    public void adjustV4Angle(double newAngle){
        armAngle.setTargetAngle(newAngle, 1.0); //also feels off verify later - James
    }
}
