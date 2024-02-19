package frc.robot;

import edu.wpi.first.wpilibj.XboxController;

public final class ControlBoard {

    static XboxController xboxController = new XboxController(0);

    public static double getXvelocityDrive(){
        return xboxController.getRawAxis(3)-xboxController.getRawAxis(2);
    }
    
}
