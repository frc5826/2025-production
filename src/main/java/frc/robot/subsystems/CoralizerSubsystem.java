package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.math.MathHelper;
import frc.robot.math.PID;

import static frc.robot.Constants.*;
import static frc.robot.Constants.Coralizer.*;

public class CoralizerSubsystem extends LoggedSubsystem{

    private SparkMax wristMotor;
    private boolean starting;

    private PID pid;
    private PID normalPID;
    private PID algaePID;

    private double wristTarget;

    private Timer wristEnableTimer;

    private double gConstant;

    private ShooterSubsystem shooterSubsystem;

    public CoralizerSubsystem(ShooterSubsystem shooterSubsystem) {
        this.shooterSubsystem = shooterSubsystem;

        starting = false;
        wristMotor = new SparkMax(3, SparkLowLevel.MotorType.kBrushless);
        gConstant = cCoralizerG;
        normalPID = new PID(cCoralizerP, cCoralizerI, cCoralizerD, cCoralizerMax, cCoralizerMin, 0, this::getRotation);
        algaePID = new PID(cCoralizerPAlgae, cCoralizerIAlgae, cCoralizerDAlgae, cCoralizerMax, cCoralizerMin, 0, this::getRotation);
        pid = normalPID;
        SmartDashboard.putString("coralizer/PIDName", "normal PID");
        wristTarget = 30;
        pid.setGoal(wristTarget);
        SparkMaxConfig config = new SparkMaxConfig();
        config.idleMode(SparkBaseConfig.IdleMode.kBrake);
        config.inverted(false);
        config.smartCurrentLimit(40);
        config.signals.absoluteEncoderPositionPeriodMs(20);
//        config.absoluteEncoder.inverted(true);
//        config.absoluteEncoder.zeroCentered(true);
        config.absoluteEncoder.zeroOffset(304 / 360.0);
        wristMotor.configure(config, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters);

        SmartDashboard.putData("coralizer/PID", pid);
        SmartDashboard.putNumber("coralizer/Encoder", 0.0);
        SmartDashboard.putNumber("coralizer/Output", 0.0);

        wristEnableTimer = new Timer();
        wristEnableTimer.reset();
        wristEnableTimer.stop();
    }

    @Override
    public void periodic() {

        if (shooterSubsystem.hasAlgae() && !shooterSubsystem.hasCoral() && pid == normalPID) {
            pid = algaePID;
            pid.setGoal(normalPID.getGoal());
            gConstant = cCoralizerGAlgae;
            SmartDashboard.putString("coralizer/PIDName", "algae PID");
        } else if((!shooterSubsystem.hasAlgae() || shooterSubsystem.hasCoral()) && pid == algaePID) {
            pid = normalPID;
            pid.setGoal(algaePID.getGoal());
            gConstant = cCoralizerG;
            SmartDashboard.putString("coralizer/PIDName", "normal PID");
        }

        double speed = pid.calculate() + gConstant * Math.cos(getRotation() * (Math.PI / 180));

        if(starting) {
            if(!DriverStation.isTest()){
                starting = false;
            }
            wristMotor.set(0.03);
        } else {
            wristMotor.set(speed);
        }
        SmartDashboard.putNumber("coralizer/Output", speed);

        SmartDashboard.putNumber("coralizer/Encoder", getRotation());
    }

//    public void setALgaePID(boolean algae) {
//        if (algae) {
//            gConstant = cCoralizerGAlgae;
//            pid = algaePID;
//        } else {
//            //gConstant = cCoralizerG;
//            //pid = normalPID;
//        }
//    }

    public void setWristTarget(double wristTarget) {
        pid.setGoal(MathHelper.clamp(wristTarget, cCoralizerMinRotation, cCoralizerMaxRotation));
        this.wristTarget = wristTarget;
    }

    public double getWristTarget() {
        return wristTarget;
    }

    public double getRotation() {
        double position = wristMotor.getAbsoluteEncoder().getPosition() * 360;
        if (position >= 180){
            position -= 360;
        }
        return position;
    }

    public void startWristTimer(){
        wristEnableTimer.restart();
    }

    public void setStarting() {
        starting = true;
    }
}
