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
        shootingSpeedMap.put(4.58, 37.5);
        shootingSpeedMap.put(3.63,33.5);
        shootingSpeedMap.put(2.52,31.5);
        shootingSpeedMap.put(1.56,27.5);
        shootingSpeedMap.put(5.895, 40.75);

        timeOfflightMap.put(4.58, 1.22);
        timeOfflightMap.put(3.63, 1.14);
        timeOfflightMap.put(2.52, 0.80);
        timeOfflightMap.put(1.56, 0.57);

        //Needs to be inialized with data from testing
        //In form of speed , Time Of Flight
    }

    public double getShotSpeed(double distance){
        return shootingSpeedMap.get(distance);
    }

    public double getShotTime(double distance){
        return timeOfflightMap.get(distance);
    }

}
