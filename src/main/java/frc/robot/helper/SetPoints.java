package frc.robot.helper;

public enum SetPoints {
    Home(0,0),//a
    Intake(0,0),//b
    PlaceHigh(0,0),//x
    PlaceMid(0,0),//y
    PlaceLow(0,0),//rb
    GrabSubstation(0,0),//lb
    GrabCubeAuto(0,0),
    GrabConeAuto(0,0),
    PlaceCubeAuto(0,0),
    PlaceCone2Auto(0,0);
    public double x;
    public double y;
    private SetPoints(double x, double y){
        this.x = x;
        this.y = y;
    }
}
