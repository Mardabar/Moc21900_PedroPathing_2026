package subsystems;

import com.qualcomm.robotcore.hardware.DcMotorEx;

public class ShootSystembad {

    private final String mode;

    private DcMotorEx rs;

    // CONSTANTS

    private final double OVERSHOOT_VEL_MULT = 1.68;
    private final double OVERSHOOT_ANG_MULT = 1;
    private final double ANGLE_CONST = 2.08833333;
    private final int ELBOW_GEAR_RATIO = 4;
    private final double MAX_HEIGHT = 1.4;

    // SHOOT VARS

    public boolean shootPrep;
    public boolean shootReady;
    private double shootAngle;
    private double shootVel;

    // FEEDING VARS

    private double feedDur = 200;
    private double ascendDur = 900;
    private double retDur = 600;

    public void setShootPos(double dist){
        /* dist is the total distance the ball will travel until it hits the ground
           it's multiplied by 1.3 because the ball will hit the goal first, so using the
           equation, it'll be about 1 meter high (the height of the goal) when it hit our requested distance
         */
        dist *= 1.3;

        // The angle and velocity are both calculated using the distance we found
        shootAngle = ((distToAngle(dist) * OVERSHOOT_ANG_MULT) - 53.5);
        shootVel = angleToVel(distToAngle(dist)) * OVERSHOOT_VEL_MULT;

        shootPrep = false;
        shootReady = true;
    }

    public void setShootPos(double ix, double iy, double fx, double fy){
    /* dist is the total distance the ball will travel until it hits the ground
       It's divided by 40 to turn the field units into meters
       Then, it's multiplied by 1.3 because the ball will hit the goal first, so using
       equation, it'll be about 1 meter high (the height of the goal) when it hit our r
     */
        double dist = (Math.sqrt(Math.pow(fx - ix, 2) + Math.pow(fy - iy, 2)) / 40) * 1.3;

        // The angle and velocity are both calculated using the distance we found
        shootAngle = ((distToAngle(dist) * OVERSHOOT_ANG_MULT) - 53.5);
        shootVel = angleToVel(distToAngle(dist)) * OVERSHOOT_VEL_MULT;
    }

    // GETTERS

    public double getAngleEnc(){
        return angleToEncoder(shootAngle);
    }

    public double getShootVel(){
        return velToRot(shootVel);
    }

    public double getShootAngle(){
        return shootAngle;
    }

    public double getFeedDur(){
        return feedDur;
    }

    public double getAscendDur(){
        return ascendDur;
    }

    public double getRetDur(){
        return retDur;
    }

    // CONVERSIONS

    public double distToAngle(double dist){
        return Math.toDegrees(Math.atan(54.88 / (9.8 * dist)));
    }

    // This function translates angle to velocity using the already set maximum height
    public double angleToVel(double angle){
        return Math.sqrt((MAX_HEIGHT * 19.6) / Math.pow(Math.sin(Math.toRadians(angle)), 2));
    }

    // This function translates velocity to motor power specifically for 6000 RPM motors combined with 72 mm Gecko Wheels
    public double velToRot(double vel){
        return (vel / (7.2 * Math.PI)) * 2800;
    }

    // This function translates an angle in degrees to an encoder value on 223 RPM motors
    public double angleToEncoder(double angle){
        return angle * ANGLE_CONST * ELBOW_GEAR_RATIO;
    }

    public ShootSystembad(String mode){
        if (mode.equals("teleop"))
            this.mode = mode;
        else if (mode.equals("autonomous"))
            this.mode = mode;
        else
            this.mode = "invalid mode input";
    }
}
