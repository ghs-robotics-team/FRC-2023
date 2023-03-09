package frc.robot.helper;

public enum SetPoints {
    Home(-0.01673539425,-0.02193501098,-0.01673539425,-0.02193501098),//Dpad right
    PlaceHigh(-0.41571247730,-0.67332323554,-0.39509674671,-0.489049655632),//Y
    PlaceMid(-0.32039507436,-0.31796631412,-0.33955555392,-0.23608453798),//B
    PlaceLow(-0.39509674671,-0.07747285111,-0.39509674671,-0.07747285111),//A
    GrabSubstation(-0.34173843047,-0.37346761621,-0.34173843047,-0.37346761621),//Dpad up
    GrabIntake(-0.17923803373,-1.23205096840,-0.17923803373,-1.23205096840),//Dpad left
    GrabGround(-0.51903994881,-0.50021812110,-0.51903994881,-0.50021812110),//Dpad down
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
