package org.firstinspires.ftc.teamcode.subsystems.intake;

import static java.lang.Math.max;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.utils.priority.PriorityCRServo;
import org.firstinspires.ftc.teamcode.utils.priority.PriorityMotor;
import org.firstinspires.ftc.teamcode.utils.priority.PriorityServo;

@Config
public class Intake {
    public enum IntakeRollerState {
        ON,
        OFF,
        REVERSE
    }

    public static double antiStallRampStep = 0.1;

    public final Robot robot;
    public final PriorityCRServo intakeRollerServo;
    private IntakeRollerState intakeRollerState = IntakeRollerState.OFF;

    public Intake(@NonNull Robot robot) {
        this.robot = robot;
        intakeRollerServo = new PriorityCRServo(
                robot.hardwareMap.get(CRServo.class, "intakeRollerServo"),
                "intakeRollerServo",
                3, 5
        );
        robot.hardwareQueue.addDevice(intakeRollerServo);
    }

    public void update() {
        switch(intakeRollerState){
            case OFF:
                intakeRollerServo.setTargetPower(0.0);
                break;
            case ON:
                intakeRollerServo.setTargetPower(1.0);
                break;
            case REVERSE:
                intakeRollerServo.setTargetPower(-1.0);
                break;
            default:
                throw new IllegalStateException("Unexpected value for intakeMotorState: " + intakeRollerState);
        }
    }

    public IntakeRollerState getIntakeRollerState() { return intakeRollerState; }

    public void setOff() { intakeRollerState = IntakeRollerState.OFF; }

    public void setOn() { intakeRollerState = IntakeRollerState.ON; }

    public void setReverse() { intakeRollerState = IntakeRollerState.REVERSE; }
}
