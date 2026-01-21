package frc.robot.utils;

/** A utility class for the Walled Lake Robotics CAN ID standard. */
public class CANIDs {
  private CANIDs() {
    throw new UnsupportedOperationException("This is a utility class!");
  }

  /**
   * The roboRIO.
   *
   * @return The CAN ID.
   */
  public static int roboRIO() {
    return 0;
  }

  /**
   * The power distribution hub (PDH).
   *
   * @return The CAN ID.
   */
  public static int PDH() {
    return 1;
  }

  /**
   * The front left drive motor.
   *
   * @return The CAN ID.
   */
  public static int frontLeftDrive() {
    return 2;
  }

  /**
   * The front right drive motor.
   *
   * @return The CAN ID.
   */
  public static int frontRightDrive() {
    return 4;
  }

  /**
   * The rear right drive motor.
   *
   * @return The CAN ID.
   */
  public static int rearRightDrive() {
    return 6;
  }

  /**
   * The rear left drive motor.
   *
   * @return The CAN ID.
   */
  public static int rearLeftDrive() {
    return 8;
  }

  /**
   * The front left turning motor.
   *
   * @return The CAN ID.
   * @apiNote Only used in swerve drive.
   */
  public static int frontLeftTurning() {
    return frontLeftDrive() + 1;
  }

  /**
   * The front right turning motor.
   *
   * @return The CAN ID.
   * @apiNote Only used in swerve drive.
   */
  public static int frontRightTurning() {
    return frontRightDrive() + 1;
  }

  /**
   * The rear right turning motor.
   *
   * @return The CAN ID.
   * @apiNote Only used in swerve drive.
   */
  public static int rearRightTurning() {
    return rearRightDrive() + 1;
  }

  /**
   * The rear left turning motor.
   *
   * @return The CAN ID.
   * @apiNote Only used in swerve drive.
   */
  public static int rearLeftTurning() {
    return rearLeftDrive() + 1;
  }

  /**
   * The front left absolute encoder.
   *
   * @return The CAN ID.
   * @apiNote Only used in swerve drive.
   */
  public static int frontLeftEncoder() {
    return 10 + frontLeftTurning();
  }

  /**
   * The front right absolute encoder.
   *
   * @return The CAN ID.
   * @apiNote Only used in swerve drive.
   */
  public static int frontRightEncoder() {
    return 10 + frontRightTurning();
  }

  /**
   * The rear right absolute encoder.
   *
   * @return The CAN ID.
   * @apiNote Only used in swerve drive.
   */
  public static int rearRightEncoder() {
    return 10 + rearRightTurning();
  }

  /**
   * The rear left absolute encoder.
   *
   * @return The CAN ID.
   * @apiNote Only used in swerve drive.
   */
  public static int rearLeftEncoder() {
    return 10 + rearLeftTurning();
  }

  /**
   * A secondary (non-drive) motor.
   *
   * @param deviceNum The device number from 0-9.
   * @return The CAN ID.
   */
  public static int secondaryMotor(int deviceNum) {
    return 20 + deviceNum;
  }

  /**
   * The pneumatics module.
   *
   * @return The CAN ID.
   */
  public static int pneumaticsModule() {
    return pneumaticsModule(0);
  }

  /**
   * A pneumatics module.
   *
   * @param moduleNum The module number form 0-9. Defaults to 0.
   * @return The CAN ID.
   */
  public static int pneumaticsModule(int moduleNum) {
    return 30 + moduleNum;
  }

  /**
   * A miscellaneous CAN device.
   *
   * @param deviceNum The device number from 0-9.
   * @return The CAN ID.
   */
  public static int miscellaneousDevice(int deviceNum) {
    return 40 + deviceNum;
  }
}
