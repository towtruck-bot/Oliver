package org.firstinspires.ftc.teamcode.subsystems.intake;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.utils.PID;
import org.firstinspires.ftc.teamcode.utils.Utils;
import org.firstinspires.ftc.teamcode.utils.priority.PriorityMotor;
import org.firstinspires.ftc.teamcode.utils.priority.PriorityServo;

@Config
public class Intake {
    public enum IntakeRollerState {
        ON,
        OFF,
        KEEP_IN,
        UNJAM,
        REVERSE,
    }

    public enum IntakeState {
        START_EXTENDING,
        EXTENDING,
        DROP_DOWN,
        EXTENDED,
        PICK_UP,
        RETRACTING,
        FINISH_RETRACTING,
        IDLE,
    }

    public final Robot robot;
    public final PriorityMotor intakeRollerMotor;
    public final PriorityMotor intakeExtensionMotor;
    public final PriorityServo intakeFlipServo;

    private IntakeState intakeState = IntakeState.RETRACTING;

    private IntakeRollerState intakeRollerState = IntakeRollerState.OFF;
    public static double keepBlockInPower = 0.2; // TODO Replace this placeholder
    public static long unjamDuration = 500; // milliseconds
    private long unjamLastTime;
    public IntakeRollerState motorState = IntakeRollerState.OFF;


    public static double extensionMaxPosition = 21; // TODO Replace this placeholder value with actual limit
    public static double extensionPositionTolerance = 0.25; // TODO Replace this placeholder
    public static double extendingStartFlipPosition = 3; // TODO Replace this placeholder
    public static double extendedMinPosition = 5; // TODO Replace this placeholder
    public static double distanceToIntake = 5; // TODO Replace this placeholder
    private double targetPositionWhenExtended = extendedMinPosition;
    private double extensionCurrentPosition = 0;
    private double extensionControlTargetPosition = 0;
    public static PID pid = new PID(0.2, 0, 0.02); // TODO Replace these placeholders

    public static double flipDownAngle = Math.toRadians(180);
    public static double flipAngleToGoOverBarrier = Math.toRadians(90);

    /**
     * Initializes the intake. Uses motors intakeRollerMotor and intakeExtensionMotor. -- Daniel
     * @param robot the robot (must have robot.sensors defined)
     */
    public Intake(@NonNull Robot robot) {
        this.robot = robot;

        this.intakeRollerMotor = new PriorityMotor(
                this.robot.hardwareMap.get(DcMotorEx.class, "intakeRollerMotor"),
                "intakeRollerMotor",
                1, 2, this.robot.sensors
        );
        this.robot.hardwareQueue.addDevice(intakeRollerMotor);

        this.intakeExtensionMotor = new PriorityMotor(
                this.robot.hardwareMap.get(DcMotorEx.class, "intakeExtensionMotor"),
                "intakeExtensionMotor",
                1, 2, this.robot.sensors
        );
        this.robot.hardwareQueue.addDevice(intakeExtensionMotor);

        this.intakeFlipServo = new PriorityServo(
                this.robot.hardwareMap.get(Servo.class, "intakeFlipServo"),
                "intakeFlipServo",
                PriorityServo.ServoType.AXON_MINI,
                1.0,
                0.0,
                1.0,
                0.0,
                false,
                1.0,
                2.0
        );
        this.robot.hardwareQueue.addDevice(intakeFlipServo);
    }

    /**
     * Updates the motors for both roller and extension. Uses PID for extension. -- Daniel
     */
    public void update() {
        long currentTime = System.nanoTime();
        this.extensionCurrentPosition = this.robot.sensors.getIntakeExtensionPosition();

        // FSM
        switch (this.intakeState) {
            case START_EXTENDING:
                this.setRollerOff();
                this.extensionControlTargetPosition = Math.min(this.targetPositionWhenExtended - distanceToIntake, extendedMinPosition);
                this.intakeFlipServo.setTargetAngle(0, 1.0);
                if (this.extensionCurrentPosition > extendingStartFlipPosition) this.intakeState = IntakeState.EXTENDING;
                else break;
            case EXTENDING:
                this.setRollerOff();
                this.extensionControlTargetPosition = Math.min(this.targetPositionWhenExtended - distanceToIntake, extendedMinPosition);
                this.intakeFlipServo.setTargetAngle(flipAngleToGoOverBarrier, 1.0);
                if (this.isExtensionAtTarget()) this.intakeState = IntakeState.DROP_DOWN;
                else break;
            case DROP_DOWN:
                this.setRollerOff();
                this.extensionControlTargetPosition = Math.min(this.targetPositionWhenExtended - distanceToIntake, extendedMinPosition);
                this.intakeFlipServo.setTargetAngle(flipDownAngle, 1.0);
                if (this.intakeFlipServo.inPosition()) {
                    this.intakeState = IntakeState.EXTENDED;
                    this.setRollerOn();
                } else break;
            case EXTENDED:
                this.extensionControlTargetPosition = this.targetPositionWhenExtended;
                this.intakeFlipServo.setTargetAngle(flipDownAngle, 1.0);
                break;
            case PICK_UP:
                this.setRollerKeepIn();
                this.extensionControlTargetPosition = this.targetPositionWhenExtended;
                this.intakeFlipServo.setTargetAngle(0, 1.0);
                if (this.intakeFlipServo.getCurrentAngle() < flipAngleToGoOverBarrier) this.intakeState = IntakeState.RETRACTING;
                else break;
            case RETRACTING:
                this.setRollerKeepIn();
                this.extensionControlTargetPosition = extendingStartFlipPosition;
                this.intakeFlipServo.setTargetAngle(0, 1.0);
                if (this.intakeFlipServo.inPosition()) this.intakeState = IntakeState.FINISH_RETRACTING;
                else break;
            case FINISH_RETRACTING:
                this.setRollerKeepIn();
                this.extensionControlTargetPosition = 0;
                this.intakeFlipServo.setTargetAngle(0, 1.0);
                if (this.isExtensionAtTarget()) {
                    this.intakeState = IntakeState.IDLE;
                    this.setRollerOff();
                } else break;
            case IDLE:
                this.extensionControlTargetPosition = 0;
                this.intakeFlipServo.setTargetAngle(0, 1.0);
                break;
        }

        // Roller control
        switch (this.intakeRollerState) {
            case OFF:
                this.intakeRollerMotor.setTargetPower(0.0);
                break;
            case ON:
                this.intakeRollerMotor.setTargetPower(1.0);
                break;
            case KEEP_IN:
                this.intakeRollerMotor.setTargetPower(keepBlockInPower);
                break;
            case UNJAM:
                this.intakeRollerMotor.setTargetPower(-1.0);
                if (currentTime > this.unjamLastTime + unjamDuration * 1e6) this.setRollerOn();
                break;
            case REVERSE:
                this.intakeRollerMotor.setTargetPower(-1.0);
                break;
        }

        // Extension control
        if (this.isExtensionAtTarget()) {
            pid.resetIntegral();
            this.intakeExtensionMotor.setTargetPower(0);
        } else {
            this.intakeExtensionMotor.setTargetPower(pid.update(this.extensionControlTargetPosition - this.extensionCurrentPosition, -1.0, 1.0));
        }
    }

    /**
     * Gets the roller's state. -- Daniel
     * @return the roller's state (ON, OFF, KEEP_IN, UNJAM, REVERSE)
     */
    public IntakeRollerState getIntakeRollerState() { return this.intakeRollerState; }

    /**
     * Sets the roller to off. -- Daniel
     */
    public void setRollerOff() { this.intakeRollerState = IntakeRollerState.OFF; }

    /**
     * Sets the roller to on. -- Daniel
     */
    public void setRollerOn() { this.intakeRollerState = IntakeRollerState.ON; }

    /**
     * Sets the roller to keep the block in. -- Daniel
     */
    public void setRollerKeepIn() { this.intakeRollerState = IntakeRollerState.KEEP_IN; }

    /**
     * Sets the roller to unjam. -- Daniel
     */
    public void setRollerUnjam() { this.intakeRollerState = IntakeRollerState.UNJAM; this.unjamLastTime = System.nanoTime(); }

    /**
     * Sets the roller to reverse. -- Daniel
     */
    public void setRollerReverse() { this.intakeRollerState = IntakeRollerState.REVERSE; }

    /**
     * Gets whether the extension is at its target position. -- Daniel
     * @return the extension target position, in inches
     */
    public boolean isExtensionAtTarget() { return Math.abs(this.extensionControlTargetPosition - this.extensionCurrentPosition) < extensionPositionTolerance; }

    /**
     * Gets the position the extension will go to when extended. -- Daniel
     * @return the extension target position, in inches
     */
    public double getTargetPositionWhenExtended() { return this.targetPositionWhenExtended; }

    /**
     * Sets the position the extension will go to when extended. The position is automatically clamped to range. -- Daniel
     * @param targetPositionWhenExtended the new extension target position, in inches
     */
    public void setTargetPositionWhenExtended(double targetPositionWhenExtended) { this.targetPositionWhenExtended = Utils.minMaxClip(targetPositionWhenExtended, extendedMinPosition, extensionMaxPosition); }

    /**
     * Gets the intake's state. -- Daniel
     * @return the intake's state (START_EXTENDING, EXTENDING, EXTENDED, PICK_UP, RETRACTING, FINISH_RETRACTING, RETRACTED)
     */
    public IntakeState getIntakeState() { return this.intakeState; }

    /**
     * Sets the state to begin extending. -- Daniel
     */
    public void extend() { this.intakeState = IntakeState.START_EXTENDING; }

    /**
     * Sets the state to begin retracting. -- Daniel
     */
    public void retract() { this.intakeState = IntakeState.PICK_UP; }
}
