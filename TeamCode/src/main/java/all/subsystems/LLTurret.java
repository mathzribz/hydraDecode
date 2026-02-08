
package all.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class LLTurret extends SubsystemBase {

    private final DcMotorEx turret;
    private final Limelight3A limelight;

    public static double kP = 0.075;
    public static double kI = 0.0;
    public static double kD = 0.004;

    private final PIDController pid;

    private boolean trackingEnabled = false;
    private double targetTx = 0.0; // ex: -3 para alinhar no centro real

    public LLTurret(HardwareMap hw) {
        turret = hw.get(DcMotorEx.class, "turret");
        limelight = hw.get(Limelight3A.class, "limelight");

        turret.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        turret.setPower(0);

        limelight.pipelineSwitch(0);
        limelight.start();

        pid = new PIDController(kP, kI, kD);
    }

    @Override
    public void periodic() {

        if (!trackingEnabled) {
            turret.setPower(0);
            pid.reset();
            return;
        }

        LLResult res = limelight.getLatestResult();
        if (res == null || !res.isValid()) {
            turret.setPower(0);
            return;
        }

        double tx = res.getTx();              // erro angular
        double power = pid.calculate(tx, targetTx);

        power = clamp(power, -0.5, 0.5);      // limite seguro

        turret.setPower(power);
    }


    public void enableTracking(double targetTx) {
        this.targetTx = targetTx;
        trackingEnabled = true;
    }

    public void disableTracking() {
        trackingEnabled = false;
        turret.setPower(0);
        pid.reset();
    }

    public void manual(double power) {
        trackingEnabled = false;
        turret.setPower(power);
    }

    private double clamp(double v, double min, double max) {
        return Math.max(min, Math.min(max, v));
    }
}
