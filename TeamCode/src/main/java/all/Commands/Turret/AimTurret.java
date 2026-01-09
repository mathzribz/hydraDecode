package all.Commands.Turret;

import com.arcrobotics.ftclib.command.CommandBase;
import com.pedropathing.geometry.Pose;
import all.Configs.Turret.FieldConstants;
import all.Subsystems.DrivePose;
import all.Subsystems.Turret;
import all.Configs.Turret.MatchContext;
import all.Configs.Turret.Alliance;


public class AimTurret extends CommandBase {

    private final Turret turret;
    private final DrivePose drive;

    public AimTurret(Turret t, DrivePose d) {
        turret = t;
        drive = d;
        addRequirements(turret);
    }

    @Override
    public void execute() {
        Pose robot = drive.getPose();
        Pose goal = (MatchContext.alliance == Alliance.BLUE)
                ? FieldConstants.BLUE_GOAL
                : FieldConstants.RED_GOAL;

        double dx = goal.getX() - robot.getX();
        double dy = goal.getY() - robot.getY();

        double angle = Math.toDegrees(Math.atan2(dy, dx)) - robot.getHeading();
        turret.setTarget(angle);
    }
}
