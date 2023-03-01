package frc.robot;

public enum AutoType{
    Top(0),
    Middle(1),
    Bottom(2);
    public int id;
    private AutoType(int id){
        this.id = id;
    }
}