package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;


/// wrapped PID class
public class PID extends LinearOpMode{
    private long last_error,sum_error;
    private double k_P,k_I,k_D;
    private double edge,B;


    public void runOpMode() {
    }

    /// initialize
    public PID(double p,double i,double d,double b){
        last_error=0;
        sum_error=0;
        edge = -1;
        k_P = p;
        k_I = i;
        k_D = d;
        B=b;
    }

    /// set output limitations
    public void limit(double a){
        edge = a;
    }

    /// PID calculation
    public double run(int current,int target){
        int error = target-current;
        if(Math.abs(error)<B) {
            error =0;
        }
        sum_error += error;

        double output = k_P * error + k_I * sum_error + k_D * (error - last_error);
        last_error=error;

        if(edge != -1){
            if(output > edge)
                output = edge;
            if(output < -edge)
                output = -edge;
        }


        return output;
    }




}
