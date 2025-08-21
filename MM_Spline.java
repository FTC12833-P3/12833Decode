package org.firstinspires.ftc.teamcode;

public class MM_Spline {
    private final double[] xPoints;
    private final double[] yPoints;
    private final double[] sectionLengths;
    private double fullCurveLength = 0;
    private double distanceTraveled = 0;

    public MM_Spline(double[] xHandles, double[] yHandles, int sections){
        this.xPoints = new double[(int)(sections + 1)];
        this.yPoints = new double[(int)(sections + 1)];
        sectionLengths = new double[sections + 1];
        for (int i = 0; i <= sections; i++){
            double t = (1.0 /sections) * i;
            this.xPoints[i] = xHandles[1] + Math.pow((1-t), 2) * (xHandles[0] - xHandles[1]) + Math.pow(t, 2) * (xHandles[2] - xHandles[1]);
            this.yPoints[i] = yHandles[1] + Math.pow((1-t), 2) * (yHandles[0] - yHandles[1]) + Math.pow(t, 2) * (yHandles[2] - yHandles[1]);
            if (i >0){
                sectionLengths[i - 1] = Math.hypot(this.xPoints[i-1] - this.xPoints[i], this.yPoints[i-1] - this.yPoints[i]);
                fullCurveLength += sectionLengths[i - 1];
            }
        }
        sectionLengths[sections] = Math.hypot(this.xPoints[sections -1] - this.xPoints[sections], this.yPoints[sections-1] - this.yPoints[sections]);
        fullCurveLength += sectionLengths[sections];
    }

    public MM_Spline(double[] xHandles, double[] yHandles, int sections, boolean isCubic) {
        this.xPoints = new double[sections + 1];
        this.yPoints = new double[sections + 1];
        sectionLengths = new double[sections + 1];
        int splinePointIndex = 0;

        for (int c = 0; c <= xHandles.length / 4; c += (sections + 1)){

            for (int i = 0; i <= sections; i++) {
                double t = (1.0 / sections) * i;

                this.xPoints[i + c] = Math.pow((1 - t), 3) * xHandles[splinePointIndex] + 3 * Math.pow((1 - t), 2) * t * xHandles[splinePointIndex + 1] + 3 * (1 - t) * Math.pow(t, 2) * xHandles[splinePointIndex + 2] + Math.pow(t, 3) * xHandles[splinePointIndex + 3];
                this.yPoints[i + c] = Math.pow((1 - t), 3) * yHandles[splinePointIndex] + 3 * Math.pow((1 - t), 2) * t * yHandles[splinePointIndex + 1] + 3 * (1 - t) * Math.pow(t, 2) * yHandles[splinePointIndex + 2] + Math.pow(t, 3) * yHandles[splinePointIndex + 3];
                if (i > 0) {
                    sectionLengths[(i + c) - 1] = Math.hypot(this.xPoints[(i + c) - 1] - this.xPoints[(i + c)], this.yPoints[(i + c) - 1] - this.yPoints[(i + c)]);
                    fullCurveLength += sectionLengths[(i + c) - 1];
                }
            }
            splinePointIndex += 4;
            sectionLengths[sections + c] = Math.hypot(this.xPoints[(sections + c) - 1] - this.xPoints[sections + c], this.yPoints[(sections + c) - 1] - this.yPoints[sections + c]);
            fullCurveLength += sectionLengths[sections + c];
        }
    }

    public double[] getNextPoint(int currentSection){
        return new double[]{xPoints[currentSection], yPoints[currentSection]};
    }

    public void updateDistanceTraveled(int currentSection){
        distanceTraveled += sectionLengths[currentSection];
    }

    public double getError(){
        return fullCurveLength - distanceTraveled;
    }

    public void resetDistanceTraveled(){
        distanceTraveled = 0;
    }

    public boolean splineDone(int currentSection) {
        return true;
    }

    public double[] getxPoints(){
        return xPoints;
    }
    public double[] getyPoints(){
        return yPoints;
    }
}
