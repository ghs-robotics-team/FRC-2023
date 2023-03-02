package frc.robot.commands.misc;

public class InverseKinematics {
    private double r = 0;
    private double theta = 0;
    private double l1 = 26.5;
    private double l2 = 27;
    private double shoulderAngle = 0;//opposite of l2
    private double elbowAngle = 0;//opposite of h
    private boolean right = true;
    private double targetAngle = 0;
    private double ptargetAngle = 0;
    private double x = 0;
    private double y = 0;
    private boolean XYMode = false;
    public InverseKinematics(){}
    public void set(double r, double theta){
        this.r = r;
        this.theta = theta;
        this.XYMode = false;
        calc();
    }
    public void setXY(double x, double y){
        this.x = x;
        this.y = y;
        this.XYMode = true;
        calc();
    }
    public void move(double dr, double dtheta){
        this.r += dr;
        this.theta += dtheta;
        this.XYMode = false;
        calc();
    }
    public void calc(){
        if(!XYMode){
            x = r * Math.cos(theta);
            y = r * Math.sin(theta);
        }
        double h = Math.sqrt(x*x + y*y);
        double triangleAngle1 = Math.acos((-l2*l2 + l1*l1 + h*h) / (2*h*l1));
        double triangleAngle2 = Math.acos((-h*h + l2*l2 + l1*l1) / (2*l2*l1));
        ptargetAngle = targetAngle;
        targetAngle = Math.atan2(y,x);
        if(Math.abs(ptargetAngle) > 2*Math.PI/3 && Math.abs(targetAngle) <= 2*Math.PI/3 && !right){
            right = true;
        }
        if(Math.abs(ptargetAngle) < 2*Math.PI/3 && Math.abs(targetAngle) >= 2*Math.PI/3 && right){
            right = false;
        }
        if(right){
            shoulderAngle = targetAngle-triangleAngle1;
            elbowAngle = Math.PI-triangleAngle2;
        }else{
            shoulderAngle = targetAngle+triangleAngle1;
            elbowAngle = triangleAngle2-Math.PI;
        }
    }

    public double getShoulderAngle(){
        return shoulderAngle;
    }

    public double getElbowAngle(){
        return elbowAngle;
    }
}
