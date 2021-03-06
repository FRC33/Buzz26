package lib.drivers;

public class BuzzTalonScalars {
    private double mReduction;
    private double mDiameter;
    private int mCountsPerRev;

    public BuzzTalonScalars(double reduction, double diameter, int countsPerRev) {
        mReduction = reduction;
        mDiameter = diameter;
        mCountsPerRev = countsPerRev;
    }

    public double getReduction() {
        return mReduction;
    }
    
    public double getDiameter() {
        return mDiameter;
    }

    public double getCountsPerRev() {
        return mCountsPerRev;
    }
}