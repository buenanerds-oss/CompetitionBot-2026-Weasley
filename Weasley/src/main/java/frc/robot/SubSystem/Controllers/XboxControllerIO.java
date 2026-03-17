package frc.robot.SubSystem.Controllers;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class XboxControllerIO implements ControllerIO{

    CommandXboxController xbox;

    public XboxControllerIO(int port) {
        xbox = new CommandXboxController(port);
        
    }

    @Override
    public double getDriveX() {
        return MathUtil.applyDeadband(-xbox.getLeftY(), 0.1);//-xbox.getLeftX();
    }

    @Override
    public double getDriveY() {
        return MathUtil.applyDeadband(-xbox.getLeftX(), 0.1);//-xbox.getLeftY();
    }

    @Override
    public double getDriveTwist() {
        return MathUtil.applyDeadband(-xbox.getRightX(), 0.1);//xbox.getRightX();
    }

}
