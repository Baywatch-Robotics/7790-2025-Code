package frc.robot.subsystems.Coral;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterPivotConstants;

public class ShooterPivot extends SubsystemBase {

    private boolean isInitialized = false;

    private float desiredAngle;

    public float NormalizeAngle(float angle)
    {
        float newAngle = angle - ShooterPivotConstants.angleOffset;

         while(newAngle > 180)
         {
            newAngle -= 360;
         }

         while(newAngle < -180)
         {
            newAngle += 360;
         }
        return newAngle;
        }

        public ShooterPivot(){


    }
    @Override
        public void periodic() {
            if (!this.isInitialized) {
                this.desiredAngle = NormalizeAngle((float)(this.pivotEncoder.getAbsolutePosition().getValue()*360));
                this.isInitialized = true;
            }
    }
}
