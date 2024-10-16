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
    public final Slides slides;
    public final Drivetrain drivetrain;
    public final Intake intake;
    public final Arm arm;
    public final Hang hang;
    public RobotState state;

    public final Deposit deposit;
    public enum RobotState {
        START_DEPOSIT,
        DEPOSIT_BUCKET,
        DEPOSIT_SPECIMEN,
        RETRACT,
        EXTEND,
        INTAKE,
        GRAB,
        IDLE,
    }

    public Robot(HardwareMap hardwareMap) {
        this(hardwareMap, null);
    }

    public Robot(HardwareMap hardwareMap, Vision vision) {
        this.hardwareMap = hardwareMap;
        this.hardwareQueue = new HardwareQueue();

        this.sensors = new Sensors(this);

        this.intake = new Intake(this);
        this.drivetrain = new Drivetrain(this);
        this.slides = new Slides(this);
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
        sensors.update();

        intake.update();
        slides.update();
        drivetrain.update();
        hang.update();
        deposit.update();

        hardwareQueue.update();
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
    public void robotFSM(){
        switch(state){
            case EXTEND:
                //actuation?
                double targetExtendoValue = 5.0;
                slides.setTargetLength(targetExtendoValue); // where do i store this variable?
            case INTAKE:
                // actuation?
                intake.setRollerOn();
                if (intake.getIntakeRollerState() != Intake.IntakeRollerState.ON) { // check if intake is off
                    state = RobotState.GRAB;
                }
            case GRAB:
                // deposit function needed?
            case START_DEPOSIT:
                // deposit function needed?
            case DEPOSIT_BUCKET:
                // deposit function needed?
            case DEPOSIT_SPECIMEN:
                // deposit function needed?
            case RETRACT:
                // deposit function needed?
            case IDLE:
                intake.setRollerOff();
                slides.setTargetLength(0.0);



        }

    }

}
