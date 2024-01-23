package frc.robot.commands;

import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.Drivetrain;
import edu.wpi.first.wpilibj2.command.Command;

public class AutonomousForte extends Command {
    private Drivetrain m_drivetrain;
  private Trajectory m_trajectory;

  private final Timer m_timer = new Timer();

  private final RamseteController m_ramsete = new RamseteController();

  public AutonomousForte(Drivetrain drivetrain, Trajectory trajectory) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drivetrain);

    m_drivetrain = drivetrain;
    m_trajectory = trajectory;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_timer.reset();
    m_timer.start();
    m_drivetrain.resetOdometry(m_trajectory.getInitialPose());
    // m_ramsete.setEnabled(false);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double elapsed = m_timer.get();
    Trajectory.State reference = m_trajectory.sample(elapsed);
    ChassisSpeeds speeds = m_ramsete.calculate(m_drivetrain.getPose(), reference);
    m_drivetrain.drive(speeds);

    var t_pose = reference.poseMeters;
    var t_x = t_pose.getX();
    var t_y = t_pose.getY();
    var t_rotation = t_pose.getRotation().getDegrees();

    var a_pose = m_drivetrain.getPose();
    var a_x = a_pose.getX();
    var a_y = a_pose.getY();
    var a_rotation = a_pose.getRotation().getDegrees();

    SmartDashboard.putNumber("Pose X - Trajectory", t_x);
    SmartDashboard.putNumber("Pose Y - Trajectory", t_y);
    SmartDashboard.putNumber("Pose R - Trajectory", t_rotation);

    SmartDashboard.putNumber("Pose X - Actual", a_x);
    SmartDashboard.putNumber("Pose Y - Actual", a_y);
    SmartDashboard.putNumber("Pose R - Actual", a_rotation);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
