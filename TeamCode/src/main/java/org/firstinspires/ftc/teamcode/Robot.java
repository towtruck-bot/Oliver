package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.utils.Globals.GET_LOOP_TIME;
import static org.firstinspires.ftc.teamcode.utils.Globals.START_LOOP;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.sensors.Sensors;
import org.firstinspires.ftc.teamcode.subsystems.deposit.Arm;
import org.firstinspires.ftc.teamcode.subsystems.deposit.Deposit;
import org.firstinspires.ftc.teamcode.subsystems.deposit.Slides;
import org.firstinspires.ftc.teamcode.subsystems.drive.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.drive.Spline;
import org.firstinspires.ftc.teamcode.subsystems.hang.Hang;
import org.firstinspires.ftc.teamcode.subsystems.intake.Intake;
import org.firstinspires.ftc.teamcode.utils.Func;
import org.firstinspires.ftc.teamcode.utils.TelemetryUtil;
import org.firstinspires.ftc.teamcode.utils.priority.HardwareQueue;
import org.firstinspires.ftc.teamcode.vision.Vision;

public class Robot {
    public final HardwareMap hardwareMap;
    public final HardwareQueue hardwareQueue;
    public final Sensors sensors;
    public final Drivetrain drivetrain;
    public final Intake intake;
    public final Arm arm;
    public final Hang hang;
    public final Deposit deposit;

    public enum RobotState {
        IDLE,
        INTAKE_SAMPLE,
        TRANSFER,
        SAMPLE_READY,
        DEPOSIT_BUCKET,
        OUTTAKE,
        GRAB_SPECIMEN,
        SPECIMEN_READY,
        DEPOSIT_SPECIMEN
    }
    private RobotState state = RobotState.IDLE;
    private RobotState prevState = RobotState.IDLE;

    private boolean outtakeAndThenGrab = false;

    public Robot(HardwareMap hardwareMap) {
        this(hardwareMap, null);
    }

    public Robot(HardwareMap hardwareMap, Vision vision) {
        this.hardwareMap = hardwareMap;
        this.hardwareQueue = new HardwareQueue();

        this.sensors = new Sensors(this);

        this.intake = new Intake(this);
        this.drivetrain = new Drivetrain(this);
        this.arm = new Arm(this);
        this.hang = new Hang(hardwareMap, hardwareQueue);
        this.deposit = new Deposit(this);

        TelemetryUtil.setup();
    }

    public void update() {
        START_LOOP();
        updateSubsystems();
        updateTelemetry();
    }

    private void updateSubsystems() {
        this.sensors.update();

        this.intake.update();
        this.drivetrain.update();
        this.hang.update();
        this.deposit.update();

        this.robotFSM();

        this.hardwareQueue.update();
    }

    private void updateTelemetry() {
        TelemetryUtil.packet.put("Loop Time", GET_LOOP_TIME());
        TelemetryUtil.sendTelemetry();
    }

    public void followSpline(Spline spline, Func func) {
        long start = System.currentTimeMillis();
        drivetrain.setPath(spline);
        drivetrain.state = Drivetrain.State.GO_TO_POINT;
        drivetrain.setMaxPower(1);
        update();

        do {
            update();
        } while (((boolean) func.call()) && System.currentTimeMillis() - start <= 10000 && drivetrain.isBusy());
    }
    public void robotFSM() {
/* Main robot FSM diagram:
v------------<----------------------------------------------------<
V            ^                                                    |
IDLE > INTAKE_SAMPLE > TRANSFER > SAMPLE_READY > DEPOSIT_BUCKET >-^
^  V                                     V                        |
|  >----------------> GRAB_SPECIMEN < OUTTAKE >-------------------^
|                           V            ^
^-< DEPOSIT_SPECIMEN < SPECIMEN_READY >--^
*/
        switch (this.state) {
            case IDLE:
                // TODO Wait to continue
                break;
            case INTAKE_SAMPLE:
                if (this.prevState == RobotState.IDLE) this.intake.extend();
                if (this.intake.isRetracted()) {
                    if (this.intake.hasSample()) this.state = RobotState.TRANSFER;
                    else this.state = RobotState.IDLE;
                }
                break;
            case TRANSFER:
                if (this.prevState == RobotState.INTAKE_SAMPLE) this.deposit.startTransfer();
                if (this.deposit.isSampleReady()) this.state = RobotState.SAMPLE_READY;
                break;
            case SAMPLE_READY:
                // TODO Wait to continue
                break;
            case DEPOSIT_BUCKET:
                if (this.prevState == RobotState.SAMPLE_READY) this.deposit.startSampleDeposit();
                if (this.deposit.isSampleDepositDone()) this.state = RobotState.IDLE;
                break;
            case OUTTAKE:
                if (this.prevState == RobotState.SAMPLE_READY || this.prevState == RobotState.SPECIMEN_READY) this.deposit.startOuttake();
                if (this.deposit.isOuttakeDone()) {
                    if (this.outtakeAndThenGrab) {
                        this.state = RobotState.GRAB_SPECIMEN;
                    } else {
                        this.state = RobotState.IDLE;
                        this.outtakeAndThenGrab = true;
                    }
                }
                break;
            case GRAB_SPECIMEN:
                if (this.prevState == RobotState.IDLE || this.prevState == RobotState.OUTTAKE) this.deposit.grabSpecimen();
                if (this.deposit.isSpecimenReady()) this.state = RobotState.SPECIMEN_READY;
                break;
            case SPECIMEN_READY:
                // TODO Wait to continue
                break;
            case DEPOSIT_SPECIMEN:
                if (this.prevState == RobotState.SPECIMEN_READY) this.deposit.startSpecimenDeposit();
                if (this.deposit.isSpecimenDepositDone()) this.state = RobotState.IDLE;
                break;
        }
        prevState = state;
    }
}
