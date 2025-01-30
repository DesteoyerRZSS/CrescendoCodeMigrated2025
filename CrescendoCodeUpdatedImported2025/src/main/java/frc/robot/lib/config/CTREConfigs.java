package frc.robot.lib.config;


import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import frc.robot.Constants;
public class CTREConfigs {
    //public CANCoderConfiguration swerveCanCoderConfig;
    public CANcoderConfiguration swerveCancoderConfig;
    public MagnetSensorConfigs magnetConfig;
    public CTREConfigs() {
      //swerveCanCoderConfig = new CANCoderConfiguration();
      magnetConfig = new MagnetSensorConfigs()
        .withAbsoluteSensorDiscontinuityPoint(1)
        .withSensorDirection(Constants.Swerve.canCoderInvert)
        .withMagnetOffset(0);
      swerveCancoderConfig = new CANcoderConfiguration().withMagnetSensor(magnetConfig);
      /* Swerve CANCoder Configuration */
      //swerveCanCoderConfig.absoluteSensorRange = AbsoluteSensorRange.Unsigned_0_to_360;

      //swerveCanCoderConfig.sensorDirection = Constants.Swerve.canCoderInvert;
      //swerveCanCoderConfig.initializationStrategy =
          //SensorInitializationStrategy.BootToAbsolutePosition;
      //swerveCanCoderConfig.sensorTimeBase = SensorTimeBase.PerSecond;
    }
}
