package frc.robot;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;

public class AimingData {
    //Shot Speed required to score on hub
    private static final InterpolatingDoubleTreeMap shootingSpeedMap = new InterpolatingDoubleTreeMap();
    //Time of Flight Based on distance when scoring into hub should count from leaving shooter to crossing Hub top
    private static final InterpolatingDoubleTreeMap timeOfflightMap = new InterpolatingDoubleTreeMap();

public AimingData(){
    //Needs to be inialized with data from testing 
    //in form of distance,speed
    shootingSpeedMap.put(5.5332759667,42.0);
    //Needs to be inialized with data from testing
    //In form of speed , Time Of Flight
    timeOfflightMap.put(null,null);
}

public double getShotSpeed(double distance){
    return shootingSpeedMap.get(distance);
}

}
