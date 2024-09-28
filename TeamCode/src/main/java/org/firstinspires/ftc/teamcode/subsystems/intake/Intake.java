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
        UNJAM,
        REVERSE
    }

    public final Robot robot;
    public final PriorityMotor intakeRollerMotor;
    public final PriorityMotor intakeExtensionMotor;
    public final PriorityServo intakeFlipServo;

    private IntakeRollerState intakeRollerState = IntakeRollerState.OFF;
    public static long unjamDuration = 500; // milliseconds
    private long unjamLastTime;

    private double extensionCurrentPosition = 0;
    private double extensionTargetPosition = 0;
    public static double extensionMaxPosition = 21; // TODO Replace this placeholder value with actual limit
    public static double extensionDeadzone = 0.1; // To prevent jitter TODO Replace this placeholder
    public static PID pid = new PID(0.2, 0, 0.02); // TODO Replace these placeholders

    private double flipTargetAngle = 0;
    public static double flipDownAngle = Math.toRadians(180);
    public static double flipUpAngle = Math.toRadians(90);

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

        switch(this.intakeRollerState){
            case OFF:
                this.intakeRollerMotor.setTargetPower(0.0);
                break;
            case ON:
                this.intakeRollerMotor.setTargetPower(1.0);
                break;
            case UNJAM:
                this.intakeRollerMotor.setTargetPower(-1.0);
                if (currentTime > this.unjamLastTime + unjamDuration * 1e6) setRollerOn();
                break;
            case REVERSE:
                this.intakeRollerMotor.setTargetPower(-1.0);
                break;
        }

        this.extensionCurrentPosition = this.robot.sensors.getIntakeExtensionPosition();
        double error = this.extensionTargetPosition - this.extensionCurrentPosition;

        if (error < extensionDeadzone) {
            pid.resetIntegral();
            this.intakeExtensionMotor.setTargetPower(0);
        } else {
            this.intakeExtensionMotor.setTargetPower(pid.update(error, -1.0, 1.0));
        }

        this.intakeFlipServo.setTargetAngle(this.flipTargetAngle, 1.0);
    }

    /**
     * Gets the roller's state. -- Daniel
     * @return the roller's state (ON, OFF, UNJAM, REVERSE)
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
     * Sets the roller to unjam. -- Daniel
     */
    public void setRollerUnjam() { this.intakeRollerState = IntakeRollerState.UNJAM; this.unjamLastTime = System.nanoTime(); }

    /**
     * Sets the roller to reverse. -- Daniel
     */
    public void setRollerReverse() { this.intakeRollerState = IntakeRollerState.REVERSE; }

    /**
     * Gets the extension current position. -- Daniel
     * @return the extension current position, in inches
     */
    public double getExtensionCurrentPosition() { return this.extensionCurrentPosition; }

    /**
     * Gets the position the extension is trying to reach. -- Daniel
     * @return the extension target position, in inches
     */
    public double getExtensionTargetPosition() { return this.extensionTargetPosition; }

    /**
     * Sets the position the extension is trying to reach. The position is automatically clamped to range. -- Daniel
     * @param extensionTargetPosition the new extension target position, in inches
     */
    public void setExtensionTargetPosition(double extensionTargetPosition) { this.extensionTargetPosition = Utils.minMaxClip(extensionTargetPosition, 0, extensionMaxPosition); }

    /**
     * Gets the position the extension is trying to reach. -- Daniel
     * @return the extension target position, in inches
     */
    public double getFlipTargetAngle() { return this.flipTargetAngle; }

    /**
     * Sets the angle for the bucket flip. The position is automatically clamped to range. -- Daniel
     * @param flipTargetAngle the new extension target position, in inches
     */
    public void setFlipTargetAngle(double flipTargetAngle) { this.flipTargetAngle = Utils.minMaxClip(flipTargetAngle, 0, flipDownAngle); }
}
