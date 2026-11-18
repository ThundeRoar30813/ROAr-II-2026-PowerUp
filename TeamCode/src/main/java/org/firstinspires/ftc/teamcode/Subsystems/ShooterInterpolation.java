package org.firstinspires.ftc.teamcode.Subsystems;

import java.util.Arrays;
import java.util.Comparator;

public class ShooterInterpolation {

    private static final double[][] DATA = {
            // --- NUEVOS PUNTOS CALIBRADOS ---
            {11, 0,  1105, 5},
            {21, 2,  1065, 5},
            {36, 6,  1030, 4},
            {55, 10, 1020, 4},
            {48, 21, 1000, 3},
            {48, 24, 1000, 2},
            {6,  33, 1035, 3},
            {8,  18, 1056, 4},
            {21, 17, 1045, 4},

            // --- PUNTOS ANTERIORES QUE COMPLETAN EL MAPA ---
            {54, 4,  1075, 4},
            {60, 15, 1045, 3},
            {60, 25, 1010, 2},
            {60, 28, 995,  1},
            {24, 24, 1040, 3},
            {36, 24, 1020, 2},
            {12, 36, 1035, 3},
            {24, 36, 1025, 3},
            {36, 36, 1000, 1}
    };

    public static int NEIGHBORS = 3;
    public static double POWER = 3.0;

    public static class Result {
        public final double tps;
        public final double hoodInterpolated;
        public final int hoodLevel;
        public final double nearestDistance;

        public Result(double tps, double hood, double nearestDistance) {
            this.tps = tps;
            this.hoodInterpolated = hood;
            this.hoodLevel = Math.max(1, Math.min(5, (int) Math.round(hood)));
            this.nearestDistance = nearestDistance;
        }
    }

    private static class Neighbor {
        double distance;
        double tps;
        double hood;

        Neighbor(double distance, double tps, double hood) {
            this.distance = distance;
            this.tps = tps;
            this.hood = hood;
        }
    }

    public static Result calculate(double x, double y) {
        if (!Double.isFinite(x) || !Double.isFinite(y)) {
            throw new IllegalArgumentException("Coordenadas invalidas");
        }

        Neighbor[] neighbors = new Neighbor[DATA.length];

        for (int i = 0; i < DATA.length; i++) {
            double distance = Math.hypot(x - DATA[i][0], y - DATA[i][1]);

            if (distance < 0.5) {
                return new Result(DATA[i][2], DATA[i][3], distance);
            }

            neighbors[i] = new Neighbor(distance, DATA[i][2], DATA[i][3]);
        }

        Arrays.sort(neighbors, Comparator.comparingDouble(n -> n.distance));

        int count = Math.max(1, Math.min(NEIGHBORS, DATA.length));

        double total = 0;
        double weightedTPS = 0;
        double weightedHood = 0;

        for (int i = 0; i < count; i++) {
            Neighbor n = neighbors[i];
            double weight = 1.0 / Math.pow(n.distance, Math.max(0.1, POWER));

            total += weight;
            weightedTPS += n.tps * weight;
            weightedHood += n.hood * weight;
        }

        return new Result(
                weightedTPS / total,
                weightedHood / total,
                neighbors[0].distance
        );
    }
}