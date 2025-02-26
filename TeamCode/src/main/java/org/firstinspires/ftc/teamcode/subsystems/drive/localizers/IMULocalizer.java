package org.firstinspires.ftc.teamcode.subsystems.drive.localizers;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.sensors.Sensors;
import org.firstinspires.ftc.teamcode.subsystems.drive.OldDrivetrain;

public class IMULocalizer extends Localizer {
    public IMULocalizer(HardwareMap hardwareMap, Sensors sensors, OldDrivetrain drivetrain, String color, String expectedColor) {
        super(hardwareMap, sensors, drivetrain, color, expectedColor);
    }

    @Override
    public void update() {
        super.update();
        heading = currentPose.heading = sensors.getHeading();
    }
}
