package org.firstinspires.ftc.teamcode.opmodes;

import static org.firstinspires.ftc.teamcode.utils.Globals.START_LOOP;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.sensors.Sensors;
import org.firstinspires.ftc.teamcode.subsystems.deposit.nDeposit;
import org.firstinspires.ftc.teamcode.subsystems.drive.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.intake.nClawIntake;
import org.firstinspires.ftc.teamcode.utils.priority.HardwareQueue;

import java.util.function.BooleanSupplier;

public class RobotHW {
    public final HardwareMap hardwareMap;
    public final HardwareQueue hardwareQueue;
    public final Sensors sensors;
    public final Drivetrain drivetrain;
    public final nClawIntake nclawIntake;
    public final nDeposit ndeposit;
    private BooleanSupplier stopChecker;

    public RobotHW(HardwareMap hardwareMap, HardwareQueue hardwareQueue, Sensors sensors, Drivetrain drivetrain, nClawIntake nclawIntake, nDeposit ndeposit) {
        this.hardwareMap = hardwareMap;
        this.hardwareQueue = hardwareQueue;
        this.sensors = sensors;
        this.drivetrain = drivetrain;
        this.nclawIntake = nclawIntake;
        this.ndeposit = ndeposit;
    }
    public void update() {
        START_LOOP();

        if (this.stopChecker != null && this.stopChecker.getAsBoolean()) {
            /*Log.i("STOP", Globals.RUNMODE.toString());
            if (Globals.RUNMODE == RunMode.AUTO) {
                this.drivetrain.setBrakePad(false);
                this.drivetrain.brakePad.setForceUpdate();
                this.ndeposit.arm.armRotation.setTargetAngle(Math.toRadians(-135));
                this.ndeposit.arm.armRotation.setForceUpdate();
                this.ndeposit.arm.clawRotation.setTargetAngle(0);
                this.ndeposit.arm.clawRotation.setForceUpdate();
                this.ndeposit.arm.clawOpen();
                this.ndeposit.arm.claw.setForceUpdate();
            }*/
            this.hardwareQueue.update();
            return;
        }

        this.sensors.update();

        drivetrain.update();
        nclawIntake.update();
        ndeposit.update();
        hardwareQueue.update();

    }
}
