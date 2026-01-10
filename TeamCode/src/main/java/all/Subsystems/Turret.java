package all.Subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.control.PIDFController;
import com.pedropathing.control.PIDFCoefficients;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import com.pedropathing.geometry.Pose;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Config
public class Turret extends SubsystemBase {

    /* hardware */
    private final DcMotorEx motor;
    public final GoBildaPinpointDriver pinpoint;

    /* PID tuning */
    public static double TICKS_PER_REV = 537.7;
    public static double kpFast = 0.003;
    public static double kdFast = 0.0;
    public static double kfFast = 0.0;
    public static double kpSlow = 0.005;
    public static double kdSlow = 0.0001;
    public static double kfSlow = 0.0;
    public static double pidSwitchTicks = 30;
    public static double MAX_DEG = 180.0;

    private final PIDFController fastPID;
    private final PIDFController slowPID;

    private double targetTicks = 0;

    private boolean manualMode = false;
    private double manualPower = 0;

    public Turret(HardwareMap hw) {
        motor = hw.get(DcMotorEx.class, "t");
        motor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        motor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
        motor.setPower(0);

        // Inicializa Pinpoint (odometria)
        pinpoint = hw.get(GoBildaPinpointDriver.class, "pinpoint");
        pinpoint.update(); // garante valores iniciais

        fastPID = new PIDFController(new PIDFCoefficients(kpFast, 0, kdFast, kfFast));
        slowPID = new PIDFController(new PIDFCoefficients(kpSlow, 0, kdSlow, kfSlow));
    }

    @Override
    public void periodic() {
        // atualiza Pinpoint toda chamada de loop
        pinpoint.update();

        if (manualMode) {
            motor.setPower(manualPower);
            return;
        }

        double error = targetTicks - motor.getCurrentPosition();

        fastPID.setCoefficients(new PIDFCoefficients(kpFast, 0, kdFast, kfFast));
        slowPID.setCoefficients(new PIDFCoefficients(kpSlow, 0, kdSlow, kfSlow));

        double power;
        if (Math.abs(error) > pidSwitchTicks) {
            fastPID.updateError(error);
            fastPID.updateFeedForwardInput(Math.signum(error));
            power = fastPID.run();
        } else {
            slowPID.updateError(error);
            power = slowPID.run();
        }

        motor.setPower(power);
    }

    /**
     * Faz a turret apontar para uma pose de campo
     * @param fieldTarget pose de destino (Pedro), em polegadas
     */
    public void seguirPose(Pose fieldTarget) {
        // lê Pinpoint (mm)
        double robotXmm = pinpoint.getPosX(DistanceUnit.MM);
        double robotYmm = pinpoint.getPosY(DistanceUnit.MM);
        double robotHeadingRad = pinpoint.getHeading(AngleUnit.DEGREES);

        // converte mm → polegadas
        double robotX = robotXmm * 0.0393701;
        double robotY = robotYmm * 0.0393701;

        double dx = fieldTarget.getX() - robotX;
        double dy = fieldTarget.getY() - robotY;

        // ângulo absoluto até o alvo (radianos)
        double absoluteAngleRad = Math.atan2(dy, dx);

        // diferença entre o heading do robô e o vetor para o alvo
        double errorRad = angleWrap(absoluteAngleRad - robotHeadingRad);
        double errorDeg = Math.toDegrees(errorRad);

        setAngleDeg(errorDeg);
    }

    public void setAngleDeg(double deg) {
        double normalized = normalizeDeg(deg);
        normalized = Math.max(-MAX_DEG, Math.min(MAX_DEG, normalized));
        targetTicks = degToTicks(normalized);
        manualMode = false;
    }

    public void addAngleDeg(double delta) {
        setAngleDeg(getAngleDeg() + delta);
    }

    public double getAngleDeg() {
        return ticksToDeg(motor.getCurrentPosition());
    }

    public double getHeadingDeg() {
        return Math.toDegrees(pinpoint.getHeading(AngleUnit.DEGREES));
    }

    public void manual(double p) {
        double currentDeg = getAngleDeg();
        if ((currentDeg >= MAX_DEG && p > 0) ||
                (currentDeg <= -MAX_DEG && p < 0)) {
            motor.setPower(0); // trava nesses limites
        } else {
            motor.setPower(p);
        }
    }


    public void automatic() {
        manualMode = false;
    }

    public void resetEncoder() {
        motor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        targetTicks = 0;
    }

    /* ====== util ====== */
    private double degToTicks(double d) {
        return (d / 360.0) * TICKS_PER_REV;
    }
    private double ticksToDeg(double t) {
        return (t / TICKS_PER_REV) * 360.0;
    }
    private double normalizeDeg(double d) {
        d %= 360;
        if (d > 180)  d -= 360;
        if (d < -180) d += 360;
        return d;
    }
    private double angleWrap(double rad) {
        // mantém entre -π…π
        while (rad > Math.PI)  rad -= 2*Math.PI;
        while (rad < -Math.PI) rad += 2*Math.PI;
        return rad;
    }


}
