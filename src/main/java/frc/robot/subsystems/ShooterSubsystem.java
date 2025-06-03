package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShooterSubsystem extends SubsystemBase {

    private SparkMax intakeMotor;

    private boolean shouldHold;

    public ShooterSubsystem() {
        shouldHold = false;
        intakeMotor = new SparkMax(4, SparkLowLevel.MotorType.kBrushless);
        setIntakeSpeed(-0.2);
    }

    @Override
    public void periodic() {
        super.periodic();

        if (shouldHold){
            if (hasCoral()){
                setIntakeSpeed(-0.2);
            }
            else {
                setIntakeSpeed(-1);
            }
        }

//        if (hasAlgae()) {
//            setIntakeSpeed(0.9);
//        } else {
//            setIntakeSpeed(-0.05);
//        }

        SmartDashboard.putBoolean("coralizer/HasCoral", hasCoral());
        SmartDashboard.putBoolean("coralizer/HasAlgae", hasAlgae());
        SmartDashboard.putNumber("coralizer/IntakeSpeed", intakeMotor.getEncoder().getVelocity());
    }

    public void setIntakeSpeed(double speed) {
        intakeMotor.set(speed);
    }

    public boolean hasCoral(){
        return intakeMotor.getForwardLimitSwitch().isPressed();
    }

    public boolean hasAlgae() { return Math.abs(intakeMotor.getEncoder().getVelocity()) < 100; }

    public boolean isShouldHold() {
        return shouldHold;
    }

    public void setShouldHold(boolean shouldHold) {
        this.shouldHold = shouldHold;
    }

}
