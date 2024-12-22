package org.firstinspires.ftc.teamcode.subsystems.intake;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.sensors.Sensors;
import org.firstinspires.ftc.teamcode.utils.Globals;
import org.firstinspires.ftc.teamcode.utils.PID;
import org.firstinspires.ftc.teamcode.utils.TelemetryUtil;
import org.firstinspires.ftc.teamcode.utils.Utils;
import org.firstinspires.ftc.teamcode.utils.priority.PriorityMotor;
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
        PICKUP,
        BEGIN_RETRACT,
        RETRACTING,
        FINISH_RETRACTING,
        IDLE,
        TRANSFER,
        TESTER, // This makes the FSM not run (useful for testing)
    }

    public final Robot robot;
    public final PriorityMotor intakeRollerMotor;
    public final PriorityMotor intakeExtensionMotor;
    public final nPriorityServo intakeFlipServo;

    public IntakeState intakeState = IntakeState.FINISH_RETRACTING;

    public IntakeRollerState intakeRollerState = IntakeRollerState.OFF;
    public static double keepBlockInPower = 0.35;
    public static long unjamDuration = 300;
    private long unjamLastTime;
    private boolean didStopRoller = true;

    public static boolean autoRetractIntake = true;
    public static boolean autoUnjamIntake = true;

    public static double extMaxLen = 22;
    public static double extTolerance = 0.9;
    public static double extFlipThresh = 5;
    public double extCurrentLen = 0;
    public double extTargetLen = 0;
    private double setExtTargetLen = 0;
    public static PID pid = new PID(0.185, 0, 0.008);

    public static double flipDownAngle = Math.toRadians(-140);
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

        intakeRollerMotor = new PriorityMotor(
                robot.hardwareMap.get(DcMotorEx.class, "intakeRollerMotor"),
                "intakeRollerMotor",
                1, 3, -1.0, robot.sensors
        );
        robot.hardwareQueue.addDevice(intakeRollerMotor);

        intakeExtensionMotor = new PriorityMotor(
                robot.hardwareMap.get(DcMotorEx.class, "intakeExtensionMotor"),
                "intakeExtensionMotor",
                1, 5, robot.sensors
        );
        robot.hardwareQueue.addDevice(intakeExtensionMotor);

        intakeFlipServo = new nPriorityServo(
                new Servo[] {robot.hardwareMap.get(Servo.class, "intakeFlipServo")},
                "intakeFlipServo",
                nPriorityServo.ServoType.HITEC,
                0.19, 0.7, 0.7,
                new boolean[] {false},
                1.0, 5.0
        );
        robot.hardwareQueue.addDevice(intakeFlipServo);

        intakeFlipServo.setTargetAngle(Math.toRadians(-10), 1.0);
    }

    /**
     * Updates the motors for both roller and extension. Uses PID with deadzone for extension. -- Daniel
     */
    public void update() {
        long currentTime = System.nanoTime();
        extCurrentLen = robot.sensors.getIntakeExtensionPosition();
        sampleColor = robot.sensors.getIntakeColor();

        /*if (Globals.TESTING_DISABLE_CONTROL && Globals.RUNMODE == RunMode.TESTER) {
            pid.update(0,-1.0,1.0);
            pid.resetIntegral();
            intakeExtensionMotor.setTargetPower(0.0);
            intakeRollerMotor.setTargetPower(0.0);
            return;
        }*/

        // FSM
        switch (intakeState) {
            case START_EXTENDING:
                setRollerOff();
                extTargetLen = Math.max(setExtTargetLen, extFlipThresh);
                intakeFlipServo.setTargetAngle(0, 1.0);
                if (extCurrentLen - extFlipThresh <= extTolerance)
                    intakeState = IntakeState.EXTENDING;
                break;
            case EXTENDING:
                intakeFlipServo.setTargetAngle(flipAngleToGoOverBarrier, 1.0);
                // TODO: Write this for current robot position
                if (isExtensionAtTarget())
                    intakeState = IntakeState.DROP_DOWN;
                break;
            case DROP_DOWN:
                intakeFlipServo.setTargetAngle(flipDownAngle, 1.0);
                if (intakeFlipServo.inPosition()) {
                    intakeState = IntakeState.PICKUP;
                    sampleCheckTime = currentTime;
                    didStopRoller = false;
                    setRollerOn();
                }
                break;
            case PICKUP:
                extTargetLen = setExtTargetLen;
                intakeFlipServo.setTargetAngle(flipDownAngle, 1.0);
                if (sampleColor == Sensors.BlockColor.NONE) {
                    didStopRoller = false;
                    sampleCheckTime = currentTime;
                } else if (currentTime > sampleCheckTime + sampleConfirmDuration * 1e6) {
                    if (Globals.isRed ? sampleColor == Sensors.BlockColor.BLUE : sampleColor == Sensors.BlockColor.RED) {
                        if (autoUnjamIntake) setRollerUnjam();
                        else if (!didStopRoller) {
                            setRollerOff();
                            didStopRoller = true;
                        }
                    } else {
                        if (!didStopRoller) {
                            setRollerOff();
                            didStopRoller = true;
                        }
                        if (autoRetractIntake) intakeState = IntakeState.BEGIN_RETRACT;
                    }
                }
                break;
            case BEGIN_RETRACT:
                setRollerKeepIn();
                extTargetLen = extFlipThresh;
                intakeFlipServo.setTargetAngle(0, 1.0);
                if (intakeFlipServo.getCurrentAngle() > flipAngleToGoOverBarrier)
                    intakeState = IntakeState.RETRACTING;
                break;
            case RETRACTING:
                setRollerKeepIn();
                intakeFlipServo.setTargetAngle(0, 1.0);
                if (intakeFlipServo.inPosition())
                    intakeState = IntakeState.FINISH_RETRACTING;
                break;
            case FINISH_RETRACTING:
                setRollerKeepIn();
                extTargetLen = 0;
                if (isExtensionAtTarget())
                    intakeState = IntakeState.IDLE;
                break;
            case IDLE:
                setRollerOff();
                extTargetLen = 0;
                intakeFlipServo.setTargetAngle(0, 1.0);
                break;
            case TRANSFER:
                setRollerReverse();
                extTargetLen = 0;
                intakeFlipServo.setTargetAngle(0, 1.0);
                break;
            case TESTER:
                intakeFlipServo.setTargetAngle(0, 1.0);
                break;
        }

        // Roller control
        switch (intakeRollerState) {
            case OFF:
                intakeRollerMotor.setTargetPower(0.0);
                break;
            case ON:
                intakeRollerMotor.setTargetPower(1.0);
                break;
            case KEEP_IN:
                intakeRollerMotor.setTargetPower(keepBlockInPower);
                break;
            case UNJAM:
                intakeRollerMotor.setTargetPower(-1.0);
                if (currentTime > unjamLastTime + unjamDuration * 1e6)
                    setRollerOn();
                break;
            case REVERSE:
                intakeRollerMotor.setTargetPower(-1.0);
                break;
        }

        // Extension control
        if (isExtensionAtTarget()) {
            pid.update(0,-1.0,1.0);
            pid.resetIntegral();
            intakeExtensionMotor.setTargetPower(0.0);
        } else {
            intakeExtensionMotor.setTargetPower(pid.update(extTargetLen - extCurrentLen, -1.0, 1.0));
        }

        // Telemetry
        TelemetryUtil.packet.put("Intake.intakeState", intakeState.toString());
        TelemetryUtil.packet.put("Intake.extensionControlTargetPosition", extTargetLen);
    }

    /**
     * Gets the roller's state. -- Daniel
     * @return the roller's state (ON, OFF, KEEP_IN, UNJAM, REVERSE)
     */
    public IntakeRollerState getIntakeRollerState() { return intakeRollerState; }

    /**
     * Sets the roller to off. -- Daniel
     */
    public void setRollerOff() { intakeRollerState = IntakeRollerState.OFF; }

    /**
     * Sets the roller to on. -- Daniel
     */
    public void setRollerOn() { intakeRollerState = IntakeRollerState.ON; }

    /**
     * Sets the roller to keep the block in. -- Daniel
     */
    public void setRollerKeepIn() { intakeRollerState = IntakeRollerState.KEEP_IN; }

    /**
     * Sets the roller to unjam. -- Daniel
     */
    public void setRollerUnjam() { intakeRollerState = IntakeRollerState.UNJAM; unjamLastTime = System.nanoTime(); }

    /**
     * Sets the roller to reverse. -- Daniel
     */
    public void setRollerReverse() { intakeRollerState = IntakeRollerState.REVERSE; }

    /**
     * Checks if the extension is at its target position. -- Daniel
     * @return whether the extension is at its target position
     */
    public boolean isExtensionAtTarget() {
        return Math.abs(extTargetLen - extCurrentLen) <= extTolerance;
    }

    /**
     * Gets the intake's state. -- Daniel
     * @return the intake's state (START_EXTENDING, EXTENDING, EXTENDED, PICK_UP, RETRACTING, FINISH_RETRACTING, RETRACTED, IDLE, TRANSFER)
     */
    public IntakeState getIntakeState() { return intakeState; }

    /**
     * Sets the state to begin extending. -- Daniel
     */
    public void extend() {
        if (intakeState == IntakeState.IDLE)
            intakeState = IntakeState.START_EXTENDING;
    }

    /**
     * Sets the state to begin retracting, or exit transfer. -- Daniel
     */
    public void retract() {
        if (intakeState == IntakeState.BEGIN_RETRACT)
            intakeState = IntakeState.RETRACTING;
    }

    /**
     * Sets the state to transfer. -- Daniel
     */
    public void transfer() {
        if (intakeState == IntakeState.IDLE) intakeState = IntakeState.TRANSFER;
    }

    /**
     * Checks if the intake is retracted. -- Daniel
     * @return whether the intake is retracted
     */
    public boolean isRetracted() { return intakeState == IntakeState.IDLE; }

    /**
     * Checks if the intake has a sample in it. -- Daniel
     * @return whether the intake has a sample in it
     */
    public boolean hasSample() { return sampleColor != Sensors.BlockColor.NONE; }

    public void setExtTargetLength(double length) {
        setExtTargetLen = Math.min(length, extMaxLen);
    }

    public double getExtTargetLen() {
        return setExtTargetLen;
    }
}