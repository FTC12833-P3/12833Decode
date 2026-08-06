package org.firstinspires.ftc.teamcode;

public class MM_Spline {
    private final double[] xPoints; //TODO make finals all caps
    private final double[] yPoints;
    private final double[] sectionLengths;
    private double fullCurveLength = 0; //TODO change curve to be spline and spline to be curve
    private double distanceTraveled = 0; //what is difference b/w fullCurveLength and distanceTraveled??

    public MM_Spline(double[] xHandles, double[] yHandles, int sections){
        this.xPoints = new double[(int)(sections + 1)]; //can remove "this". on xPoints and yPoints
        this.yPoints = new double[(int)(sections + 1)];
        sectionLengths = new double[sections + 1];
        for (int i = 0; i <= sections; i++){ //incrementing along sections of spline
            double t = (1.0 /sections) * i;
            this.xPoints[i] = xHandles[1] + Math.pow((1-t), 2) * (xHandles[0] - xHandles[1]) + Math.pow(t, 2) * (xHandles[2] - xHandles[1]); //equation for quadratic Bezier curve
            this.yPoints[i] = yHandles[1] + Math.pow((1-t), 2) * (yHandles[0] - yHandles[1]) + Math.pow(t, 2) * (yHandles[2] - yHandles[1]);
            if (i > 0){
                sectionLengths[i - 1] = Math.hypot(this.xPoints[i-1] - this.xPoints[i], this.yPoints[i-1] - this.yPoints[i]);
                fullCurveLength += sectionLengths[i - 1];
            }
        }
        sectionLengths[sections] = Math.hypot(this.xPoints[sections -1] - this.xPoints[sections], this.yPoints[sections-1] - this.yPoints[sections]);
        fullCurveLength += sectionLengths[sections];
    }

    public MM_Spline(double[] xHandles, double[] yHandles, int sections, boolean isCubic) { //overloaded method for cubic Bézier curve
        this.xPoints = new double[sections + 1];
        this.yPoints = new double[sections + 1];
        sectionLengths = new double[sections + 1];
        int splinePointIndex = 0; //point/section along spline

        for (int c = 0; c <= xHandles.length / 4; c += (sections + 1)){ // checking for completed spline; change c to j probably

            for (int i = 0; i <= sections; i++) { //incrementing along sections of spline
                double t = (1.0 / sections) * i; //t goes from 0 to 1, inclusive

                this.xPoints[i + c] = Math.pow((1 - t), 3) * xHandles[splinePointIndex] + 3 * Math.pow((1 - t), 2) * t * xHandles[splinePointIndex + 1] + 3 * (1 - t) * Math.pow(t, 2) * xHandles[splinePointIndex + 2] + Math.pow(t, 3) * xHandles[splinePointIndex + 3]; //cubic Bézier curve formula; can remove "this." from x and y points
                this.yPoints[i + c] = Math.pow((1 - t), 3) * yHandles[splinePointIndex] + 3 * Math.pow((1 - t), 2) * t * yHandles[splinePointIndex + 1] + 3 * (1 - t) * Math.pow(t, 2) * yHandles[splinePointIndex + 2] + Math.pow(t, 3) * yHandles[splinePointIndex + 3]; //i = point along individual spline, c = # spline you are on
                if (i > 0) { //if past start of spline, aka if en route
                    sectionLengths[(i + c) - 1] = Math.hypot(this.xPoints[(i + c) - 1] - this.xPoints[(i + c)], this.yPoints[(i + c) - 1] - this.yPoints[(i + c)]); //find hypotenuse b/w difference b/w penultimate and last x points and difference b/w penultimate and last y points; get rid of all "this."
                    fullCurveLength += sectionLengths[(i + c) - 1]; //presumably the -1s are so there is no error from going beyond array limit? make that cleaner
                }
            }
            splinePointIndex += 4; //there are 4 control points, adding 4 means that you have skipped from beginning to end of spline, now spline is complete
            sectionLengths[sections + c] = Math.hypot(this.xPoints[(sections + c) - 1] - this.xPoints[sections + c], this.yPoints[(sections + c) - 1] - this.yPoints[sections + c]); //find hypotenuse b/w difference b/w penultimate and last x points and difference b/w penultimate and last y points; get rid of all "this."
            fullCurveLength += sectionLengths[sections + c]; //fullCurveLength includes all individual splines that look like one movement
        }
    }

    public double[] getNextPoint(int currentSection){ //not sure if this is named correctly? might work out b/c array starts at 0 so currentSection is 1 ahead
        return new double[]{xPoints[currentSection], yPoints[currentSection]};
    }

    public void updateDistanceTraveled(int currentSection){
        distanceTraveled += sectionLengths[currentSection];
    }

    public void setLastPoint(double x){
        this.xPoints[MM_Autos.SPLINE_DETAIL_LEVEL] = x;
    }

    public double getError(){
        return fullCurveLength - distanceTraveled;
    } //not using??

    public void resetDistanceTraveled(){
        distanceTraveled = 0;
    }

    public boolean splineDone(int currentSection) {
        return true;
    } //not using??

    public double[] getxPoints(){
        return xPoints;
    }
    public double[] getyPoints(){
        return yPoints;
    } //not using this but using getxPoints??
}
