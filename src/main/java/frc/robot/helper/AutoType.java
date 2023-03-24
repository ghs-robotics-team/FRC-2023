package frc.robot.helper;

public enum AutoType{
    BASIC(0),
    Middle(1),
    Bottom(2);
    public int id;
    private AutoType(int id){
        this.id = id;
    }
}