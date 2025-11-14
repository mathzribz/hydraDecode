package all.subsystems;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DistanceSensor;

public class SensorDD extends OpMode {
    DistanceSensor dd;

    @Override
    public void init() {
        dd = hardwareMap.get(DistanceSensor.class, "dd");
    }

    @Override
    public void loop() {

    }

}
