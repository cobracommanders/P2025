package frc.robot.subsystems.LED;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;

public class LED {
    private Patterns patterns;
    private Color color;
    public AddressableLED led;
    public AddressableLEDBuffer ledBuffer;
    private static final double FAST_BLINK_DURATION = 0.08;
    private static final double SLOW_BLINK_DURATION = 0.25;
    private final Timer blinkTimer = new Timer();
    
    public LED(){
        led = new AddressableLED(0);
        ledBuffer = new AddressableLEDBuffer(60);
        led.setLength(ledBuffer.getLength());
        led.setData(ledBuffer);
        led.start();

        for (var i = 0; i < ledBuffer.getLength(); i++) {
            // Sets the specified LED to the RGB values for red
            ledBuffer.setRGB(i, 77, 198, 225);
         }
         led.setData(ledBuffer);

        // if DriverStation.isDisabled(){

        // }else{

        // }


        double time = blinkTimer.get();
        double onDuration = 0;
        double offDuration = 0;
  
        if (patterns == Patterns.FAST_BLINK) {
          onDuration = FAST_BLINK_DURATION;
          offDuration = FAST_BLINK_DURATION * 2;
        } else if (patterns == Patterns.SLOW_BLINK) {
          onDuration = SLOW_BLINK_DURATION;
          offDuration = SLOW_BLINK_DURATION * 2;
    }
}
}
