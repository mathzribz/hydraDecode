//package all.Tests;
//
//import com.pedropathing.telemetry.SelectScope;
//import com.pedropathing.telemetry.SelectableOpMode;
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//import com.qualcomm.robotcore.eventloop.opmode.OpMode;
//
//import org.firstinspires.ftc.robotcore.external.function.Consumer;
//import all.Tests.SwitchCaseAuto;
//import java.util.function.Supplier;
//
//
//
//@Autonomous
//public class SelectTeleOp extends SelectableOpMode {
//
//    public SelectTeleOp() {
//        super("Select a Tuning OpMode", s -> {
//            s.folder("Localization", l -> {
//                l.add("Localization Test", SwitchCaseAuto::new);
//                l.add("Forward Tuner", testServo::new);
//
//            });
//});
//
//}
//}