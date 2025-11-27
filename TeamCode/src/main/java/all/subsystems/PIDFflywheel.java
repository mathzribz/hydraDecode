package all.subsystems;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import com.arcrobotics.ftclib.controller.PIDFController;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.VoltageSensor;

public class PIDFflywheel extends OpMode {

    private VoltageSensor vs;
    public static double FlywheelSpeed = 0.6;

    private DcMotorEx Flywheel, Flywheel2;

    public static double kP = 0.0006;
    public static double kI = 0.0;
    public static double kD = 0.00001;
    public static double kF = 0.0015;

    public static double TICKS_PER_REV = 28;

    private static final PIDFController pidf =  new PIDFController(kP, kI, kD, kF);;



    /** Converte RPM desejado em TPS */
    private static double rpmToTPS(double rpm) {
        return (rpm / 60.0) * TICKS_PER_REV;
    }

    /**
     * Calcula e retorna potência final do flywheel.
     *
     * @param currentVelocity velocidade real (TPS)
     * @param targetRPM rotação alvo desejada
     */
    public static double compute(double currentVelocity, double targetRPM) {

        double targetTPS = rpmToTPS(targetRPM);

        double output = pidf.calculate(currentVelocity, targetTPS);

        // Limita entre -1 e 1
        return Math.max(-1, Math.min(1, output));
    }

    private static MotorEx Fly;
    static double vL = Fly.getVelocity();
    double vR = Flywheel2.getVelocity();
    static double vAvg = vL;

    public static double power = PIDFflywheel.compute(vAvg, 1200);

    @Override
    public void init() {
        vs = hardwareMap.voltageSensor.iterator().next();
    }

    @Override
    public void loop() {
        double voltage = vs.getVoltage();

        if (voltage > 13){
            FlywheelSpeed = 0.575;
        }

    }
}