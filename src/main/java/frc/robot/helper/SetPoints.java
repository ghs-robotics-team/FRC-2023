package frc.robot.helper;

public enum SetPoints {
    Home(0,0,0,0),//Dpad right
    PlaceHigh(0,0,0,0),//Y
    PlaceMid(0,0,0,0),//B
    PlaceLow(0,0,0,0),//A
    GrabSubstation(0,0,0,0),//Dpad up
    GrabIntake(0,0,0,0),//Dpad left
    GrabGround(0,0,0,0),//Dpad down
    GrabCubeAuto(0,0,0,0),//only auto
    PlaceCubeAuto(0,0,0,0),//only auto
    PlaceCone2Auto(0,0,0,0);//only auto
    public double coneS;
    public double coneE;
    public double cubeS;
    public double cubeE;
    private static boolean cubeMode = false;
    /***
     * 
     * @param coneS The angle the shoulder has to be at if we are in cone mode
     * @param coneE The angle the elbow has to be at if we are in cone mode
     * @param cubeS The angle the shoulder has to be at if we are in cube mode
     * @param cubeE The angle the elbow has to be at if we are in cube mode
     */
    private SetPoints(double coneS, double coneE, double cubeS, double cubeE){
        this.coneS = coneS;
        this.coneE = coneE;
        this.cubeS = cubeS;
        this.cubeE = cubeE;
    }
    public double getShoulderAngle(){
        if(cubeMode){
            return cubeS;
        }else{
            return coneS;
        }
    }
    public double getElbowAngle(){
        if(cubeMode){
            return cubeE;
        }else{
            return coneE;
        }
    }
    public static void setCubeMode(boolean mode){
        cubeMode = mode;
    }
}
