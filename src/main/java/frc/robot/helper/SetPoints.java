package frc.robot.helper;

public enum SetPoints {
    Home(-6,6),//a
    Intake(-26,-10),//b
    PlaceHigh(48,25),//x
    PlaceMid(32,10),//y
    PlaceLow(16,0),//rb
    GrabSubstation(16,25),//lb
    GrabCubeAuto(48,-10),
    PlaceCubeAuto(48,25),
    PlaceCone2Auto(52,25);
    public double x;
    public double y;
    private SetPoints(double x, double y){
        this.x = x;
        this.y = y;
    }
}
