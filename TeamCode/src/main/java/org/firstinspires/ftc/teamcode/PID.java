package org.firstinspires.ftc.teamcode;

// For timer function
import com.qualcomm.robotcore.util.ElapsedTime;

public class PID {
    private double p_coeff;
    private double i_coeff;
    private double d_coeff;
    private double p_val = 0.0;
    private double d_val = 0.0;
    private double i_val = 0.0;

    private double maxPow = 100.0;

    private double dt = 0.0;

    private double currentError = 0.0;
    private double previousError = 0.0;

    ElapsedTime timer = new ElapsedTime();

    public PID() {
        this.p_coeff = 0.1;
        this.i_coeff = 0.1;
        this.d_coeff = 0.1;
    }

    public PID(double p, double i, double d) {
        this.p_coeff = p;
        this.i_coeff = i;
        this.d_coeff = d;
    }

    public double power() {
        return 0.01 * (p_coeff * p_val + i_coeff * i_val + d_coeff * d_val);
    }

    public void setError(double error) {
        // Calculate dt and restart timer for the next dt interval.
        if (timer.milliseconds() != 0.0) dt = timer.milliseconds();
        timer.reset();

        previousError = currentError;
        currentError = error;

        p_val = currentError;
        i_val += (currentError * dt);
        // Integral saturation
        if (i_val > maxPow) i_val = maxPow;
        if (i_val < -maxPow) i_val = -maxPow;
        d_val = (currentError - previousError) / dt;
    }

    public double getP() {return p_val;}

    public double getI() {return i_val;}

    public double getD() {return d_val;}
//
//    public void setProportionalCoeff(double p) {
//        this.p_coeff = p;
//    }
//
//    public void setIntegralCoeff(double i) {
//        this.i_coeff = i;
//    }
//
//    public void setDerivativeCoeff(double d) {
//        this.d_coeff = d;
//    }
}
