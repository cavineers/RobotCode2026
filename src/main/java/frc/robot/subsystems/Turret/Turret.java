package frc.robot.subsystems.Turret;

import static frc.robot.subsystems.Turret.TurretConstants.*;

import java.util.function.Supplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.controller.PIDController;


public class Turret extends SubsystemBase{
    private final TurretIO io;
    private final TurretIOInputsAutoLogged inputs = new TurretIOInputsAutoLogged();
    private PIDController turretPID = new PIDController(0.0, 0.0, 0.0); //TODO: input real terms
    private Supplier<Pose2d> robotPoseSupplier;
    private Supplier<Pose2d> targetPoseSupplier;

    
        public Turret(TurretIO io) {
            this.io = io;
        }


        public void setPoseSuppliers(Supplier<Pose2d> robotPoseSupplier, Supplier<Pose2d> targetPoseSupplier){
         this.robotPoseSupplier = robotPoseSupplier;
         this.targetPoseSupplier = targetPoseSupplier;
        }

    //field relative positioning
        public void periodic() {
            io.updateInputs(inputs);

            if(robotPoseSupplier == null || targetPoseSupplier == null){
                io.setTurretVoltage(0.0);
                return;
            }
    
        turretPID = new PIDController(0.0, 0.0, 0.0); //TODO: input real terms

        Pose2d robotPose = robotPoseSupplier.get();
        Pose2d targetPose = targetPoseSupplier.get();

        Pose2d robotToTarget = targetPose.relativeTo(robotPose);

        double fieldAngle = 
            Math.atan2(robotToTarget.getY(), robotToTarget.getX());

        fieldAngle = MathUtil.clamp(fieldAngle, kMinAngleRad, kMaxAngleRad);

         double output = turretPID.calculate(inputs.turretPositionRad, fieldAngle);
    }

    public Command goToPresetCommand(double rotations) {
         return Commands.run(() -> {
            this.io.setClosedLoop(true);
            io.updateTurretPosition(rotations);
        }, this);
       
    }

    public boolean getTurretLimitSwitch(){
        return inputs.turretLimitSwitchPressed;
    }

    public Command setTurretVoltageCommand(double volts){
        return Commands.run(() -> io.setTurretVoltage(volts), this).finallyDo(interrupted -> io.setTurretVoltage(0));
    }

    public Command resetTurretPositionCommand(){
        return Commands.run(() -> io.resetTurretPosition(), this).finallyDo(interrupted -> io.setTurretVoltage(0));
    }

    public Command rotateCommand(){
        return Commands.run(() -> io.rotate(), this).finallyDo(interrupted -> io.setTurretVoltage(0.0));
    }

    public void setTurretVoltage(double volts){
        io.setTurretVoltage(volts);
    }
    
}
