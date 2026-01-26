package all.Commands;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import com.arcrobotics.ftclib.command.CommandBase;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;

import all.Configs.Pedro.Constants;
import all.subsystems.Drive;
import all.subsystems.Turret;


import java.util.function.Supplier;

public class t extends CommandBase {



    private final Turret turret;
    private final Drive drive;
    private final Supplier<Pose> targetPose;

    public t(
            Turret turret,
            Drive drive,
            Supplier<Pose> targetPose
    ) {
        this.turret = turret;
        this.drive = drive;
        this.targetPose = targetPose;


        addRequirements(turret);
    }

    @Override
    public void execute() {

        turret.seguirPose(targetPose.get());
    }

    @Override
    public boolean isFinished() {
        return false; // roda o tempo todo
    }
}