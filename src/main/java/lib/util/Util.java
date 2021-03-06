package lib.util;

import lib.geometry.Rotation2d;

import java.util.List;
import java.util.function.Supplier;

import com.ctre.phoenix.ErrorCode;

/**
 * Contains basic functions that are used often.
 */
public class Util {
    public static final double kEpsilon = 1e-12;

    /**
     * Prevent this class from being instantiated.
     */
    private Util() {}

    /**
     * Limits the given input to the given magnitude.
     */
    public static double limit(double v, double maxMagnitude) {
        return limit(v, -maxMagnitude, maxMagnitude);
    }

    public static double limit(double v, double min, double max) {
        return Math.min(max, Math.max(min, v));
    }

    public static boolean inRange(double v, double maxMagnitude) {
        return inRange(v, -maxMagnitude, maxMagnitude);
    }

    /**
     * Checks if the given input is within the range (min, max), both exclusive.
     */
    public static boolean inRange(double v, double min, double max) {
        return v > min && v < max;
    }

    public static double interpolate(double a, double b, double x) {
        x = limit(x, 0.0, 1.0);
        return a + (b - a) * x;
    }

    public static String joinStrings(final String delim, final List<?> strings) {
        StringBuilder sb = new StringBuilder();
        for (int i = 0; i < strings.size(); ++i) {
            sb.append(strings.get(i).toString());
            if (i < strings.size() - 1) {
                sb.append(delim);
            }
        }
        return sb.toString();
    }

    public static boolean epsilonEquals(double a, double b, double epsilon) {
        return (a - epsilon <= b) && (a + epsilon >= b);
    }

    public static boolean epsilonEquals(double a, double b) {
        return epsilonEquals(a, b, kEpsilon);
    }

    public static boolean epsilonEquals(int a, int b, int epsilon) {
        return (a - epsilon <= b) && (a + epsilon >= b);
    }

    public static boolean allCloseTo(final List<Double> list, double value, double epsilon) {
        boolean result = true;
        for (Double value_in : list) {
            result &= epsilonEquals(value_in, value, epsilon);
        }
        return result;
    }

    public static double toTurretSafeAngleDegrees(Rotation2d rotation2d) {
        double result = rotation2d.getDegrees() % 360.0;
        if (result > 270) {
            result -= 360;
        } else if (result < -90) {
            result += 360;
        }
        return result;
    }

    //region Additional Buzz functions
    public static double avg(double a, double b) {
        return (a + b) / 2d;
    }

    public static double conditionalAvg(double a, boolean aValid, double b, boolean bValid) {
        return (aValid && bValid) ? avg(a, b) : (aValid ? a : (bValid ? b : 0));
    }

    public static double round(double arg0) {
        return round(arg0, 2);
    }

    public static double round(double arg0, int decimalPlaces) {
        double scale = Math.pow(10, decimalPlaces);
        return Math.round(arg0 * scale) / scale;
    }
    //endregion

    //From https://github.com/frc4362/gemlib/blob/master/src/main/java/com/gemsrobotics/lib/drivers/motorcontrol/GemTalon.java
    public static synchronized boolean runWithRetries(final Supplier<ErrorCode> call) {
		boolean success;

		int tries = 0;

		int MAX_TRIES = 5;
        do {
			success = call.get() == ErrorCode.OK;
		} while (!success && tries++ < MAX_TRIES);

		if (tries >= MAX_TRIES || !success) {
			return false;
		} else {
			return true;
		}
	}
}
