package cowlib;

/** Add your docs here. */
public class Util {
  private Util() {
  }

  /**
   * @param x the value to be mapped
   * @param a the beginning of the input range
   * @param b the end of the input range
   * @param c the beginning of the output range
   * @param d the end of the input range
   * @return value x within [a,b] mapped to [c,d]
   *         return = (X-A)/(B-A) * (D-C) + C
   */
  public static final double mapDouble(double x, double a, double b, double c, double d) {
    return (x - a) / (b - a) * (d - c) + c;
  }

  /**
   * @param xy double array of length 2, to be squared by it's magnitude
   *
   *           squares a 2d vector
   */
  public static final double[] square2DVector(double[] xy) {
    // I'd prefer to copy xy here into a new variable here, but I think that'd be
    // wasteful
    // so this method just modified the input and returns it

    // Converting to Polar coordinates (uses coordinates (r, theta) where `r` is
    // magnitude and `theta` is the angle relative to 0. Usually 0 is in the
    // positive direction of a cartesian graph's x axis, and increases positively
    // with counterclockwise rotation).
    double r = Math.sqrt(xy[0] * xy[0] + xy[1] * xy[1]);
    double theta = Math.atan2(xy[1], xy[0]);

    // Square radius and scale by max velocity. This allows for slower speed when
    // the drivestick is closer to the center without limiting the max speed because
    // 1 is the max output of the drivestick and 1 * 1 = 1.
    r = r * r;

    // Convert to Cartesian coordinates (uses coordinates (x,y)) by getting the `x`
    // and `y` legs of the right triangle where `r` is the hypotenuse and `x` and
    // `y` are the legs. Learn trigonometry *shrug*. Also multiplies by 0.5 if the
    // elevator is in use so the robot has smaller chances of tipping.
    xy[0] = r * Math.cos(theta);
    xy[1] = r * Math.sin(theta);

    return xy;
  }
}
