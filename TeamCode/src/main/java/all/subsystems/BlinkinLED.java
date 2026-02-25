package all.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class BlinkinLED extends SubsystemBase {

    private RevBlinkinLedDriver blink;

   public BlinkinLED(HardwareMap hw){
       blink = hw.get(RevBlinkinLedDriver.class,"blink");


   }



   public void orange(){
       blink.setPattern(RevBlinkinLedDriver.BlinkinPattern.ORANGE);

   }

   public void red(){
       blink.setPattern(RevBlinkinLedDriver.BlinkinPattern.RED);

   }

   public void violet(){
       blink.setPattern(RevBlinkinLedDriver.BlinkinPattern.VIOLET);


   }

   public void black(){
       blink.setPattern(RevBlinkinLedDriver.BlinkinPattern.BLACK);


   }


}
