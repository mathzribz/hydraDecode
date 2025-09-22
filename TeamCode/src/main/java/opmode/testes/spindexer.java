package opmode.testes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.Servo;
import android.graphics.Color;

@TeleOp
public class spindexer extends LinearOpMode {

    private NormalizedColorSensor intakeSensor;
    private NormalizedColorSensor outSensor;
    private Servo spindexer;

    private String[] slots = {"empty", "empty", "empty"};
    private final double[] slotPositions = {0.0, 0.33, 0.66};

    @Override
    public void runOpMode() throws InterruptedException {
        intakeSensor = hardwareMap.get(NormalizedColorSensor.class, "intakeSensor");
        outSensor = hardwareMap.get(NormalizedColorSensor.class, "outSensor");
        spindexer = hardwareMap.get(Servo.class, "spindexer");

        spindexer.setPosition(0.0);

        waitForStart();

        while (opModeIsActive()) {
            String intakeColor = detectSampleColor(intakeSensor);
            if (!intakeColor.equals("unknown")) {
                for (int i = 0; i < slots.length; i++) {
                    if (slots[i].equals("empty")) {
                        slots[i] = intakeColor;
                        int nextSlot = (i + 1) % slots.length;
                        spindexer.setPosition(slotPositions[nextSlot]);
                        sleep(500);
                        break;
                    }
                }
            }

            String outColor = detectSampleColor(outSensor);
            if (!outColor.equals("unknown")) {
                for (int i = 0; i < slots.length; i++) {
                    if (slots[i].equals(outColor)) {
                        slots[i] = "empty";
                        break;
                    }
                }
            }

            telemetry.addData("Slot 1", slots[0]);
            telemetry.addData("Slot 2", slots[1]);
            telemetry.addData("Slot 3", slots[2]);
            telemetry.addData("Servo Pos", spindexer.getPosition());
            telemetry.update();
        }
    }

    private String detectSampleColor(NormalizedColorSensor sensor) {
        float[] hsv = new float[3];
        Color.colorToHSV(sensor.getNormalizedColors().toColor(), hsv);
        float hue = hsv[0];
        float sat = hsv[1];
        float val = hsv[2];

        if (hue >= 100 && hue <= 150 && sat > 0.4 && val > 0.3) {
            return "green";
        }
        if (hue >= 270 && hue <= 310 && sat > 0.4 && val > 0.3) {
            return "purple";
        }
        return "unknown";
    }
}
