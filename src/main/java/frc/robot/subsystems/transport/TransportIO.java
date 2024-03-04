package frc.robot.subsystems.transport;

public interface TransportIO {
  /**
   * Sets the transport speeds. Speeds are how fast the belt/edge of intake wheels will move at, in meters per second.
   * This is effectively the speed that the note moves.  
   * If the intake is running faster than the transport, the transport will run at the intake speed.
   * @param transportSpeedMetersPerSecond The speed that the upper transport wheels will spin.
   * @param intakeSpeedMetersPerSecond The speed that the intake wheels will spin.
   */
  public default void setTransportSpeed(double transportSpeedMetersPerSecond, double intakeSpeedMetersPerSecond) {}
}