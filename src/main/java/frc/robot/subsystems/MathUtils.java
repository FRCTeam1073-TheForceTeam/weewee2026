package frc.robot.subsystems;

public class MathUtils 
{
    public static double wrapAngleRadians(double angle)
    {
        angle = (angle + Math.PI) % (2 * Math.PI);
        if (angle < 0)
        {
            angle += 2 * Math.PI;
        }
        return angle - Math.PI;
    } 

    public static double wrapAngleDegrees(double angle)
    {
        angle = (angle + 180) % (360);
        if (angle < 0)
        {
            angle += 360;
        }
        return angle - 180;
    } 
}
