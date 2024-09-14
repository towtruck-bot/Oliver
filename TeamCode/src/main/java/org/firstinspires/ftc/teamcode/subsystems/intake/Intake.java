package org.firstinspires.ftc.teamcode.subsystems.intake;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.CRServo;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.utils.priority.PriorityCRServo;

@Config
public class Intake {
    public enum IntakeRollerState {
        ON,
        OFF,
        REVERSE
    }

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
