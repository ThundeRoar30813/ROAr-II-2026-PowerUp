package org.firstinspires.ftc.teamcode.Subsystems;

import java.util.Arrays;
import java.util.Comparator;

public class ShooterInterpolation {

    // { x_in, y_in, target_tps, hood_level }
    // Pega aquí las filas de BioBuzzCalibrationV4.csv donde sí entró el tiro
    private static final double[][] DATA = {
            {12, 12, 1070, 5},
            {24, 12, 1080, 5},
            {36, 12, 1060, 4},
            {12, 24, 1080, 4},
            {24, 24, 1040, 3},
            {36, 24, 1020, 2},
            {12, 36, 1035, 3},
            {24, 36, 1025, 3},
            {36, 36, 1000, 1},
            {60, 28, 995, 1},
            {60, 25, 1010, 2},
            {60, 15, 1045, 3},
            {57, 10, 1065, 3},
            {54, 4, 1075, 4}
    };

    // 3 vecinos evita promediar puntos del otro lado de la cancha
    public static int NEIGHBORS = 3;
    // Potencia 3.0 da prioridad fuerte al punto calibrado más cercano
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

            if (distance < 0.5) { // Si estás a menos de media pulgada de un punto calibrado, úsalo directo
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
            double weight = 1.0 / Math.pow(n.distance, POWER);

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