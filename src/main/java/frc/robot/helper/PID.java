package frc.robot.helper;

public class PID {
    public double kp = 0;
    public double ki = 0;
    public double kd = 0;
    double reset = 0;
    double previousError = 0;
    public PID (double kp, double ki, double kd){
    this.kd = kd;
    this.ki = ki;
    this.kp = kp;
    }
    public double step(double error) {
        reset += error;
        double result = kp * error + ki * reset + kd * (error - previousError);
        previousError = error;
        return result;
    }
    public void reset(){
        reset = 0;
        previousError = 0;
        
    }
}


