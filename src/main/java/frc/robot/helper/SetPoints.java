package frc.robot.helper;

public enum SetPoints {
    Home(-0.179519656,0.118331277,-0.179519656,0.118331277),//Dpad right
    PlaceHigh(-2.84735099247,2.33453468146,-2.54981878025,1.6791672965),//Y
    PlaceMid(-1.98633589544,1.30311667930,-2.03453941553,0.86007234815),//B
    PlaceLow(-1.879955545,0.4018722,-1.879955545,0.4018722),//A
    GrabSubstation(-2.25893525846,1.42107980189,-2.25893525846,1.42107980189),//Dpad up
    GrabIntake(-0.18949276339,1.855533,-0.18949276339,1.855533),//Dpad left
    GrabGround(-3.621983086,2.07111950057,-2.9603800131,0.624514258436),//Dpad down
    GrabCubeAuto(0,0,0,0),//only auto
    PlaceCubeAuto(-2.84735099247,2.33453468146,-2.54981878025,1.6791672965),//only auto
    PlaceCone2Auto(0,0,0,0);//only auto
    public double coneS;
    public double coneE;
    public double cubeS;
    public double cubeE;
    public static boolean cubeMode = false;
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
