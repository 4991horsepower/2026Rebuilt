package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShooterOptimizer extends SubsystemBase {

    public boolean willHitTarget(Translation2d pos, Translation2d robotVel, 
                             double z0, double zt, double V, 
                             double thetaDeg, double phiDeg, 
                             Translation2d targetXY, double threshold) {
    double g = 9.80665;
    double phiRad = Math.toRadians(phiDeg);
    double thetaRad = Math.toRadians(thetaDeg);
    
    double vz = V * Math.sin(phiRad);
    double hDiff = zt - z0;
    double discriminant = (vz * vz) - 2 * g * hDiff;

    if (discriminant <= 0) return false; 

    double t = (vz + Math.sqrt(discriminant)) / g;
    if (t < 1e-6) return false;

    double netVx = robotVel.getX() + V * Math.cos(phiRad) * Math.cos(thetaRad);
    double netVy = robotVel.getY() + V * Math.cos(phiRad) * Math.sin(thetaRad);

    double x_pred = pos.getX() + netVx * t;
    double y_pred = pos.getY() + netVy * t;

    return targetXY.getDistance(new Translation2d(x_pred, y_pred)) <= threshold;
}

public double[] optimizeShot(Translation2d pos, Translation2d robotVel, 
                             double z0, double zt, double V, 
                             double thetaDeg, double phiDeg, 
                             Translation2d targetXY, int maxIters) {
    double learningRate = 0.05; 
    double g = 9.80665;
    double currentV = V;
    double currentPhiRad = Math.toRadians(phiDeg);
    double thetaRad = Math.toRadians(thetaDeg);

    for (int i = 0; i < maxIters; i++) {
        double vz = currentV * Math.sin(currentPhiRad);
        double D = (vz * vz) - 2 * g * (zt - z0);
        
        if (D <= 1e-4) return new double[]{currentV, Math.toDegrees(currentPhiRad)};

        double sqrtD = Math.sqrt(D);
        double t = (vz + sqrtD) / g;
        
        double dt_dV = (Math.sin(currentPhiRad) / g) * (1 + (currentV * Math.sin(currentPhiRad) / sqrtD));
        double dt_dPhi = (currentV * Math.cos(currentPhiRad) / g) * (1 + (currentV * Math.sin(currentPhiRad) / sqrtD));

        double netVx = robotVel.getX() + currentV * Math.cos(currentPhiRad) * Math.cos(thetaRad);
        double netVy = robotVel.getY() + currentV * Math.cos(currentPhiRad) * Math.sin(thetaRad);
        
        double dx = (pos.getX() + netVx * t) - targetXY.getX();
        double dy = (pos.getY() + netVy * t) - targetXY.getY();

        double de_dV = 2 * dx * (Math.cos(currentPhiRad) * Math.cos(thetaRad) * (t + currentV * dt_dV)) +
                       2 * dy * (Math.cos(currentPhiRad) * Math.sin(thetaRad) * (t + currentV * dt_dV));
        
        double de_dPhi = 2 * dx * (currentV * Math.cos(thetaRad) * (Math.cos(currentPhiRad) * dt_dPhi - t * Math.sin(currentPhiRad))) +
                         2 * dy * (currentV * Math.sin(thetaRad) * (Math.cos(currentPhiRad) * dt_dPhi - t * Math.sin(currentPhiRad)));

        currentV -= de_dV * learningRate;
        currentPhiRad -= de_dPhi * learningRate;
    }
    return new double[]{currentV, Math.toDegrees(currentPhiRad)};
}

public double calculateThetaDeg(Translation2d pos, Translation2d targetXY, 
                                Translation2d robotVel, double V, 
                                double phiDeg, double t) {
    double phiRad = Math.toRadians(phiDeg);
    if (t < 1e-6 || Math.abs(V * Math.cos(phiRad)) < 1e-6) {
        // Return simple heading to target if robot is stationary or shot is vertical
        return Math.toDegrees(Math.atan2(targetXY.getY() - pos.getY(), targetXY.getX() - pos.getX()));
    }

    double neededVx = (targetXY.getX() - pos.getX()) / t - robotVel.getX();
    double neededVy = (targetXY.getY() - pos.getY()) / t - robotVel.getY();
    
    return Math.toDegrees(Math.atan2(neededVy, neededVx));
}

}
