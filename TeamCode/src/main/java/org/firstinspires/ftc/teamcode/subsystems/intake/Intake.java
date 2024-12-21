package org.firstinspires.ftc.teamcode.subsystems.intake;

import android.util.Log;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.sensors.Sensors;
import org.firstinspires.ftc.teamcode.utils.Globals;
import org.firstinspires.ftc.teamcode.utils.PID;
import org.firstinspires.ftc.teamcode.utils.RunMode;
import org.firstinspires.ftc.teamcode.utils.TelemetryUtil;
import org.firstinspires.ftc.teamcode.utils.Utils;
import org.firstinspires.ftc.teamcode.utils.priority.PriorityMotor;
import org.firstinspires.ftc.teamcode.utils.priority.PriorityServo;
import org.firstinspires.ftc.teamcode.utils.priority.nPriorityServo;

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
        TRANSFER,
    }

    public final Robot robot;
    public final PriorityMotor intakeRollerMotor;
    public final PriorityMotor intakeExtensionMotor;
    public final nPriorityServo intakeFlipServo;

    private IntakeState intakeState = IntakeState.FINISH_RETRACTING;

    private IntakeRollerState intakeRollerState = IntakeRollerState.OFF;
    public static double keepBlockInPower = 0.35;
    public static long unjamDuration = 300;
    private long unjamLastTime;
    private boolean didStopRoller = true;

    public static boolean autoRetractIntake = true;
    public static boolean autoUnjamIntake = true;

    public static double extensionMaxPosition = 22;
    public static double extensionPositionTolerance = 0.9;
    public static double extendingStartFlipPosition = 5;
    public static double extendedMinPosition = 6;
    public static double distanceToIntake = 3;
    private double targetPositionWhenExtended = 15.0;
    private double extensionCurrentPosition = 0;
    private double extensionControlTargetPosition = 0;
    public static PID pid = new PID(0.05, 0.1, 0.05);

    public static double flipDownAngle = Math.toRadians(-135);
    public static double flipAngleToGoOverBarrier = Math.toRadians(-95);

    private Sensors.BlockColor sampleColor = Sensors.BlockColor.NONE;
    private long sampleCheckTime;
    public static long sampleConfirmDuration = 100;

    /**
     * Initializes the intake. Uses motors intakeRollerMotor and intakeExtensionMotor, servo intakeFlipServo. -- Daniel
     * @param robot the robot (must have robot.sensors defined)
     */
    public Intake(@NonNull Robot robot) {
        this.robot = robot;

        this.intakeRollerMotor = new PriorityMotor(
                this.robot.hardwareMap.get(DcMotorEx.class, "intakeRollerMotor"),
                "intakeRollerMotor",
                1, 3, -1.0, this.robot.sensors
        );
        this.robot.hardwareQueue.addDevice(intakeRollerMotor);

        this.intakeExtensionMotor = new PriorityMotor(
                this.robot.hardwareMap.get(DcMotorEx.class, "intakeExtensionMotor"),
                "intakeExtensionMotor",
                1, 5, this.robot.sensors
        );
        this.robot.hardwareQueue.addDevice(intakeExtensionMotor);

        this.intakeFlipServo = new nPriorityServo(
                new Servo[]{this.robot.hardwareMap.get(Servo.class, "intakeFlipServo")},
                "intakeFlipServo",
                nPriorityServo.ServoType.HITEC,
                0.19, 0.7,
                0.7,
                new boolean[] {false},
                1.0, 5.0
        );
        this.robot.hardwareQueue.addDevice(intakeFlipServo);

        this.intakeFlipServo.setTargetAngle(Math.toRadians(-10), 1.0);
    }

    /**
     * Updates the motors for both roller and extension. Uses PID with deadzone for extension. -- Daniel
     */
    public void update() {
        long currentTime = System.nanoTime();
        this.extensionCurrentPosition = this.robot.sensors.getIntakeExtensionPosition();
        this.sampleColor = this.robot.sensors.getIntakeColor();

        if (Globals.TESTING_DISABLE_CONTROL && Globals.RUNMODE == RunMode.TESTER) {
            pid.update(0,-1.0,1.0);
            pid.resetIntegral();
            this.intakeExtensionMotor.setTargetPower(0.0);
            this.intakeRollerMotor.setTargetPower(0.0);
            return;
        }

        // FSM
        switch (this.intakeState) {
            case START_EXTENDING:
                this.setRollerOff();
                this.targetPositionWhenExtended = extendedMinPosition + distanceToIntake;
                this.extensionControlTargetPosition = Math.max(this.targetPositionWhenExtended - distanceToIntake, extendedMinPosition);
                this.intakeFlipServo.setTargetAngle(0, 1.0);
                if (this.extensionCurrentPosition >= extendingStartFlipPosition) this.intakeState = IntakeState.EXTENDING;
                else break;
            case EXTENDING:
                this.setRollerOff();
                this.extensionControlTargetPosition = Math.max(this.targetPositionWhenExtended - distanceToIntake, extendedMinPosition);
                this.intakeFlipServo.setTargetAngle(flipAngleToGoOverBarrier, 1.0);
                if (this.isExtensionAtTarget()) this.intakeState = IntakeState.DROP_DOWN;
                else break;
            case DROP_DOWN:
                this.setRollerOff();
                this.extensionControlTargetPosition = Math.max(this.targetPositionWhenExtended - distanceToIntake, extendedMinPosition);
                this.intakeFlipServo.setTargetAngle(flipDownAngle, 1.0);
                if (this.intakeFlipServo.inPosition()) {
                    this.intakeState = IntakeState.EXTENDED;
                    this.sampleCheckTime = currentTime;
                    this.didStopRoller = false;
                    this.setRollerOn();
                } else break;
            case EXTENDED:
                this.extensionControlTargetPosition = this.targetPositionWhenExtended;
                this.intakeFlipServo.setTargetAngle(flipDownAngle, 1.0);
                if (this.sampleColor == Sensors.BlockColor.NONE) {
                    this.didStopRoller = false;
                    this.sampleCheckTime = currentTime;
                } else if (currentTime > this.sampleCheckTime + sampleConfirmDuration * 1e6) {
                    if (Globals.isRed ? this.sampleColor == Sensors.BlockColor.BLUE : this.sampleColor == Sensors.BlockColor.RED) {
                        if (autoUnjamIntake) this.setRollerUnjam();
                        else if (!this.didStopRoller) {
                            this.setRollerOff();
                            this.didStopRoller = true;
                        }
                    } else {
                        if (!this.didStopRoller) {
                            this.setRollerOff();
                            this.didStopRoller = true;
                        }
                        if (autoRetractIntake) this.intakeState = IntakeState.PICK_UP;
                    }
                }
                if (this.intakeState != IntakeState.PICK_UP) break;
            case PICK_UP:
                this.setRollerKeepIn();
                this.extensionControlTargetPosition = this.targetPositionWhenExtended;
                this.intakeFlipServo.setTargetAngle(0, 1.0);
                if (this.intakeFlipServo.getCurrentAngle() > flipAngleToGoOverBarrier) this.intakeState = IntakeState.RETRACTING;
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
                if (this.isExtensionAtTarget()) this.intakeState = IntakeState.IDLE;
                else break;
            case IDLE:
                this.setRollerOff();
                this.extensionControlTargetPosition = 0;
                this.intakeFlipServo.setTargetAngle(0, 1.0);
                break;
            case TRANSFER:
                this.setRollerReverse();
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
            pid.update(0,-1.0,1.0);
            pid.resetIntegral();
            this.intakeExtensionMotor.setTargetPower(0.0);
        } else {
            this.intakeExtensionMotor.setTargetPower(pid.update(this.extensionControlTargetPosition - this.extensionCurrentPosition, -1.0, 1.0));
        }

        // Telemetry
        TelemetryUtil.packet.put("Intake.intakeState", this.intakeState.toString());
        TelemetryUtil.packet.put("Intake.targetPositionWhenExtended", this.targetPositionWhenExtended);
        TelemetryUtil.packet.put("Intake.extensionControlTargetPosition", this.extensionControlTargetPosition);
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
     * Checks if the extension is at its target position. -- Daniel
     * @return whether the extension is at its target position
     */
    public boolean isExtensionAtTarget() { return Math.abs(this.extensionControlTargetPosition - this.extensionCurrentPosition) <= extensionPositionTolerance; }

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
     * @return the intake's state (START_EXTENDING, EXTENDING, EXTENDED, PICK_UP, RETRACTING, FINISH_RETRACTING, RETRACTED, IDLE, TRANSFER)
     */
    public IntakeState getIntakeState() { return this.intakeState; }

    /**
     * Sets the state to begin extending. -- Daniel
     */
    public void extend() {
        if (this.intakeState == IntakeState.IDLE) this.intakeState = IntakeState.START_EXTENDING;
        else if (this.intakeState == IntakeState.RETRACTING) this.intakeState = IntakeState.EXTENDING;
        else if (this.intakeState == IntakeState.PICK_UP) this.intakeState = IntakeState.DROP_DOWN;
    }

    /**
     * Sets the state to begin retracting, or exit transfer. -- Daniel
     */
    public void retract() {
        if (this.intakeState == IntakeState.EXTENDED || this.intakeState == IntakeState.DROP_DOWN) this.intakeState = IntakeState.PICK_UP;
        else if (this.intakeState == IntakeState.EXTENDING) this.intakeState = IntakeState.RETRACTING;
        else if (this.intakeState == IntakeState.TRANSFER) this.intakeState = IntakeState.IDLE;
    }

    /**
     * Sets the state to transfer. -- Daniel
     */
    public void transfer() {
        if (this.intakeState == IntakeState.IDLE) this.intakeState = IntakeState.TRANSFER;
    }

    /**
     * Checks if the intake is retracted. -- Daniel
     * @return whether the intake is retracted
     */
    public boolean isRetracted() { return this.intakeState == IntakeState.IDLE; }

    /**
     * Checks if the intake has a sample in it. -- Daniel
     * @return whether the intake has a sample in it
     */
    public boolean hasSample() { return this.sampleColor != Sensors.BlockColor.NONE; }
}
