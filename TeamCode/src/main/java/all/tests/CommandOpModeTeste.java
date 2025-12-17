
package all.tests;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import all.subsystems.Commands.IntakeOff;
import all.subsystems.Commands.IntakeOn;
import all.subsystems.SubsystemsTeste;

@TeleOp
public class CommandOpModeTeste extends CommandOpMode {

    private MotorEx motor;
    private Servo servo;
    private GamepadEx gamepads1;
    private SubsystemsTeste subsystem;
    private IntakeOn IntakeOn;
    private IntakeOff IntakeOff;

    @Override
    public void initialize() {
        gamepads1 = new GamepadEx(gamepad1);

        subsystem = new SubsystemsTeste(hardwareMap, "subsystem");
        IntakeOn = new IntakeOn(subsystem);
        IntakeOff = new IntakeOff(subsystem);

        gamepads1.getGamepadButton(GamepadKeys.Button.A).whenPressed(IntakeOn);
        gamepads1.getGamepadButton(GamepadKeys.Button.B).whenPressed(IntakeOff);

    }

}
