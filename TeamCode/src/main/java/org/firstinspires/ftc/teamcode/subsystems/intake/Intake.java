package org.firstinspires.ftc.teamcode.subsystems.intake;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.utils.priority.PriorityMotor;

@Config
public class Intake {
    public enum IntakeRollerState {
        ON,
        OFF,
        REVERSE
    }

    public final Robot robot;
    public final PriorityMotor intakeRollerMotor;
    private IntakeRollerState intakeRollerState = IntakeRollerState.OFF;

    public Intake(@NonNull Robot robot) {
        this.robot = robot;
        intakeRollerMotor = new PriorityMotor(
                robot.hardwareMap.get(DcMotorEx.class, "intakeRollerMotor"),
                "intakeRollerMotor",
                1, 2, this.robot.sensors
        );
        robot.hardwareQueue.addDevice(intakeRollerMotor);
    }

    public void update() {
        switch(intakeRollerState){
            case OFF:
                intakeRollerMotor.setTargetPower(0.0);
                break;
            case ON:
                intakeRollerMotor.setTargetPower(1.0);
                break;
            case REVERSE:
                intakeRollerMotor.setTargetPower(-1.0);
                break;
            default:
                throw new IllegalStateException("Unexpected value for intakeRollerState: " + intakeRollerState);
        }
    }

    public IntakeRollerState getIntakeRollerState() { return intakeRollerState; }

    public void setOff() { intakeRollerState = IntakeRollerState.OFF; }

    public void setOn() { intakeRollerState = IntakeRollerState.ON; }

    public void setReverse() { intakeRollerState = IntakeRollerState.REVERSE; }
}
