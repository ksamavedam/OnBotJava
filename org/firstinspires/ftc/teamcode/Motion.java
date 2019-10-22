package org.firstinspires.ftc.teamcode;

public class Motion {
    private double[] values = new double[4];

    final int LENGTH = 4;

    public Motion() {
    }

    Motion(double[] values) {
        this.values = values;
    }

    Motion add(Motion other) {
        double[] out = new double[4];
        for (int i = 0; i != LENGTH; ++i) {
            out[i] = values[i] + other.getValue(i);
        }
        return new Motion(out);
    }

    public Motion scale(double x) {
        double[] out = new double[4];
        for (int i = 0; i != LENGTH; ++i) {
            out[i] = values[i] * x;
        }
        return new Motion(out);
    }

    double getValue(int i) {
        return values[i];
    }
}
