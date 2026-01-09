package all.Configs.Turret;

public class TurretConstants {

    public static final double TICKS_PER_REV = 537.7;
    public static final double DEGREES_PER_TICK = 360.0 / TICKS_PER_REV;

    public static final double MAX_ANGLE = 180;
    public static final double MIN_ANGLE = -180;

    public static double kP = 0.004;
    public static double kD = 0.0002;
    public static double kF = 0.0;

}