
package all.Tests;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import all.Commands.Intake.IntakeOff;
import all.Commands.Intake.IntakeOn;
import all.subsystems.Intake;

@TeleOp
public class CommandOpModeTeste extends CommandOpMode {

    private MotorEx motor;
    private Servo servo;
    private GamepadEx gamepads1;
    private Intake subsystem;
    private IntakeOn IntakeOn;
    private IntakeOff IntakeOff;

    @Override
    public void initialize() {
        gamepads1 = new GamepadEx(gamepad1);

        subsystem = new Intake(hardwareMap, "subsystem");
        IntakeOn = new IntakeOn(subsystem);
        IntakeOff = new IntakeOff(subsystem);

        gamepads1.getGamepadButton(GamepadKeys.Button.A).whenPressed(IntakeOn);
        gamepads1.getGamepadButton(GamepadKeys.Button.B).whenPressed(IntakeOff);


    }



}
