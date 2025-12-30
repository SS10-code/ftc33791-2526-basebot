package org.firstinspires.ftc.teamcode;

import java.util.ArrayList;
import java.util.List;

/**
 * ShooterVelocityRegression performs polynomial regression to model the relationship
 * between shooting distance and required shooter velocity.
 * 
 * <p>This class fits a polynomial of specified degree to a series of distance-velocity
 * data points using least squares regression. Once fitted, it can predict the required
 * shooter velocity for any given distance.
 * 
 * <p>Example usage:
 * <pre>{@code
 * ShooterVelocityRegression regression = new ShooterVelocityRegression(2); // 2nd degree polynomial
 * 
 * // Add points individually
 * regression.addDataPoint(24.0, 1200.0); // 24 inches, 1200 ticks/sec
 * regression.addDataPoint(48.0, 1800.0); // 48 inches, 1800 ticks/sec
 * 
 * // Or add multiple points at once
 * double[] distances = {72.0, 96.0, 120.0};
 * double[] velocities = {2400.0, 3000.0, 3600.0};
 * regression.addDataPoints(distances, velocities);
 * 
 * regression.fit();
 * double velocity = regression.predict(60.0); // Predict velocity for 60 inches
 * }</pre>
 * 
 * @author FTC Team 23070 Royal Turtles
 */
public class ShooterVelocityRegression {
    
    /**
     * Represents a single data point with distance and velocity.
     */
    public static class DataPoint {
        public final double distance;
        public final double velocity;
        
        public DataPoint(double distance, double velocity) {
            this.distance = distance;
            this.velocity = velocity;
        }
    }
    
    private final int degree;
    private final List<DataPoint> dataPoints;
    private double[] coefficients; // Polynomial coefficients [a0, a1, a2, ..., an] for a0 + a1*x + a2*x^2 + ...
    private boolean isFitted;
    
    /**
     * Creates a new polynomial regression model.
     * 
     * @param degree The degree of the polynomial to fit (e.g., 2 for quadratic, 3 for cubic)
     * @throws IllegalArgumentException if degree is less than 1
     */
    public ShooterVelocityRegression(int degree) {
        if (degree < 1) {
            throw new IllegalArgumentException("Polynomial degree must be at least 1");
        }
        this.degree = degree;
        this.dataPoints = new ArrayList<>();
        this.coefficients = null;
        this.isFitted = false;
    }
    
    /**
     * Adds a data point to the regression model.
     * 
     * @param distance The shooting distance (typically in inches)
     * @param velocity The required shooter velocity (typically in ticks per second)
     */
    public void addDataPoint(double distance, double velocity) {
        dataPoints.add(new DataPoint(distance, velocity));
        isFitted = false; // Model needs to be refitted
    }
    
    /**
     * Adds a data point to the regression model.
     * 
     * @param point The data point to add
     */
    public void addDataPoint(DataPoint point) {
        dataPoints.add(point);
        isFitted = false; // Model needs to be refitted
    }
    
    /**
     * Adds multiple data points to the regression model.
     * 
     * <p>This method accepts parallel arrays where distances[i] and velocities[i]
     * form a single data point. Both arrays must have the same length.
     * 
     * @param distances Array of shooting distances (typically in inches)
     * @param velocities Array of required shooter velocities (typically in ticks per second)
     * @throws IllegalArgumentException if the arrays have different lengths or are null
     */
    public void addDataPoints(double[] distances, double[] velocities) {
        if (distances == null || velocities == null) {
            throw new IllegalArgumentException("Distance and velocity arrays cannot be null");
        }
        if (distances.length != velocities.length) {
            throw new IllegalArgumentException("Distance and velocity arrays must have the same length");
        }
        
        for (int i = 0; i < distances.length; i++) {
            dataPoints.add(new DataPoint(distances[i], velocities[i]));
        }
        isFitted = false; // Model needs to be refitted
    }
    
    /**
     * Adds multiple data points to the regression model.
     * 
     * @param points Array of data points to add
     * @throws IllegalArgumentException if the array is null
     */
    public void addDataPoints(DataPoint[] points) {
        if (points == null) {
            throw new IllegalArgumentException("Data points array cannot be null");
        }
        
        for (DataPoint point : points) {
            if (point != null) {
                dataPoints.add(point);
            }
        }
        isFitted = false; // Model needs to be refitted
    }
    
    /**
     * Adds multiple data points to the regression model.
     * 
     * @param points List of data points to add
     * @throws IllegalArgumentException if the list is null
     */
    public void addDataPoints(List<DataPoint> points) {
        if (points == null) {
            throw new IllegalArgumentException("Data points list cannot be null");
        }
        
        for (DataPoint point : points) {
            if (point != null) {
                dataPoints.add(point);
            }
        }
        isFitted = false; // Model needs to be refitted
    }
    
    /**
     * Clears all data points and resets the model.
     */
    public void clearDataPoints() {
        dataPoints.clear();
        coefficients = null;
        isFitted = false;
    }
    
    /**
     * Gets the number of data points currently stored.
     * 
     * @return The number of data points
     */
    public int getDataPointCount() {
        return dataPoints.size();
    }
    
    /**
     * Gets the polynomial degree.
     * 
     * @return The degree of the polynomial
     */
    public int getDegree() {
        return degree;
    }
    
    /**
     * Fits the polynomial regression model to the stored data points.
     * 
     * <p>This method uses least squares regression to compute the polynomial coefficients.
     * The minimum number of data points required is (degree + 1). If fewer points are
     * available, the method will still attempt to fit but the result may be less accurate.
     * 
     * @throws IllegalStateException if no data points have been added
     */
    public void fit() {
        if (dataPoints.isEmpty()) {
            throw new IllegalStateException("Cannot fit model: no data points available");
        }
        
        int n = dataPoints.size();
        int m = degree + 1; // Number of coefficients
        
        // Build the Vandermonde matrix X and target vector y
        double[][] X = new double[n][m];
        double[] y = new double[n];
        
        for (int i = 0; i < n; i++) {
            DataPoint point = dataPoints.get(i);
            double x = point.distance;
            y[i] = point.velocity;
            
            // Build polynomial features: [1, x, x^2, x^3, ..., x^degree]
            for (int j = 0; j < m; j++) {
                X[i][j] = Math.pow(x, j);
            }
        }
        
        // Solve for coefficients using least squares: (X^T * X) * coefficients = X^T * y
        coefficients = solveLeastSquares(X, y);
        isFitted = true;
    }
    
    /**
     * Solves the least squares problem using normal equations.
     * 
     * @param X The design matrix (n x m)
     * @param y The target vector (n)
     * @return The coefficient vector (m)
     */
    private double[] solveLeastSquares(double[][] X, double[] y) {
        int n = X.length;
        int m = X[0].length;
        
        // Compute X^T * X
        double[][] XTX = new double[m][m];
        for (int i = 0; i < m; i++) {
            for (int j = 0; j < m; j++) {
                double sum = 0.0;
                for (int k = 0; k < n; k++) {
                    sum += X[k][i] * X[k][j];
                }
                XTX[i][j] = sum;
            }
        }
        
        // Compute X^T * y
        double[] XTy = new double[m];
        for (int i = 0; i < m; i++) {
            double sum = 0.0;
            for (int k = 0; k < n; k++) {
                sum += X[k][i] * y[k];
            }
            XTy[i] = sum;
        }
        
        // Solve (X^T * X) * coefficients = X^T * y using Gaussian elimination
        return solveLinearSystem(XTX, XTy);
    }
    
    /**
     * Solves a linear system using Gaussian elimination with partial pivoting.
     * 
     * @param A The coefficient matrix (n x n)
     * @param b The right-hand side vector (n)
     * @return The solution vector (n)
     */
    private double[] solveLinearSystem(double[][] A, double[] b) {
        int n = A.length;
        double[][] augmented = new double[n][n + 1];
        
        // Create augmented matrix [A | b]
        for (int i = 0; i < n; i++) {
            for (int j = 0; j < n; j++) {
                augmented[i][j] = A[i][j];
            }
            augmented[i][n] = b[i];
        }
        
        // Forward elimination with partial pivoting
        for (int i = 0; i < n; i++) {
            // Find pivot
            int maxRow = i;
            for (int k = i + 1; k < n; k++) {
                if (Math.abs(augmented[k][i]) > Math.abs(augmented[maxRow][i])) {
                    maxRow = k;
                }
            }
            
            // Swap rows
            double[] temp = augmented[i];
            augmented[i] = augmented[maxRow];
            augmented[maxRow] = temp;
            
            // Eliminate
            for (int k = i + 1; k < n; k++) {
                if (Math.abs(augmented[i][i]) < 1e-10) {
                    // Near-zero pivot, set to small value to avoid division by zero
                    augmented[i][i] = 1e-10;
                }
                double factor = augmented[k][i] / augmented[i][i];
                for (int j = i; j < n + 1; j++) {
                    augmented[k][j] -= factor * augmented[i][j];
                }
            }
        }
        
        // Back substitution
        double[] x = new double[n];
        for (int i = n - 1; i >= 0; i--) {
            x[i] = augmented[i][n];
            for (int j = i + 1; j < n; j++) {
                x[i] -= augmented[i][j] * x[j];
            }
            if (Math.abs(augmented[i][i]) < 1e-10) {
                x[i] = 0.0; // Avoid division by zero
            } else {
                x[i] /= augmented[i][i];
            }
        }
        
        return x;
    }
    
    /**
     * Predicts the required shooter velocity for a given distance.
     * 
     * <p>The model must be fitted using {@link #fit()} before predictions can be made.
     * 
     * @param distance The shooting distance (typically in inches)
     * @return The predicted shooter velocity (typically in ticks per second)
     * @throws IllegalStateException if the model has not been fitted
     */
    public double predict(double distance) {
        if (!isFitted || coefficients == null) {
            throw new IllegalStateException("Model must be fitted before making predictions. Call fit() first.");
        }
        
        double prediction = 0.0;
        for (int i = 0; i < coefficients.length; i++) {
            prediction += coefficients[i] * Math.pow(distance, i);
        }
        
        return prediction;
    }
    
    /**
     * Gets the polynomial coefficients.
     * 
     * <p>The coefficients are ordered as [a0, a1, a2, ..., an] representing
     * the polynomial: a0 + a1*x + a2*x^2 + ... + an*x^n
     * 
     * @return A copy of the coefficient array, or null if not fitted
     */
    public double[] getCoefficients() {
        if (coefficients == null) {
            return null;
        }
        return coefficients.clone();
    }
    
    /**
     * Checks if the model has been fitted.
     * 
     * @return true if the model has been fitted, false otherwise
     */
    public boolean isFitted() {
        return isFitted;
    }
    
    /**
     * Calculates the R-squared (coefficient of determination) to measure model fit quality.
     * 
     * <p>R-squared ranges from 0 to 1, where 1 indicates a perfect fit.
     * 
     * @return The R-squared value, or Double.NaN if the model is not fitted
     */
    public double getRSquared() {
        if (!isFitted || coefficients == null || dataPoints.isEmpty()) {
            return Double.NaN;
        }
        
        // Calculate mean of observed values
        double meanY = 0.0;
        for (DataPoint point : dataPoints) {
            meanY += point.velocity;
        }
        meanY /= dataPoints.size();
        
        // Calculate sum of squares
        double ssTotal = 0.0;  // Total sum of squares
        double ssResidual = 0.0; // Residual sum of squares
        
        for (DataPoint point : dataPoints) {
            double observed = point.velocity;
            double predicted = predict(point.distance);
            
            ssTotal += Math.pow(observed - meanY, 2);
            ssResidual += Math.pow(observed - predicted, 2);
        }
        
        if (ssTotal < 1e-10) {
            return 1.0; // Perfect fit (all y values are the same)
        }
        
        return 1.0 - (ssResidual / ssTotal);
    }
    
    /**
     * Gets a string representation of the fitted polynomial equation.
     * 
     * @return A string representation of the polynomial, or "Not fitted" if not fitted
     */
    @Override
    public String toString() {
        if (!isFitted || coefficients == null) {
            return "ShooterVelocityRegression[degree=" + degree + ", not fitted]";
        }
        
        StringBuilder sb = new StringBuilder("ShooterVelocityRegression[velocity = ");
        boolean first = true;
        
        for (int i = 0; i < coefficients.length; i++) {
            if (Math.abs(coefficients[i]) < 1e-6) {
                continue; // Skip near-zero coefficients
            }
            
            if (!first && coefficients[i] >= 0) {
                sb.append(" + ");
            } else if (!first) {
                sb.append(" - ");
            }
            
            if (i == 0) {
                sb.append(String.format("%.2f", coefficients[i]));
            } else if (i == 1) {
                sb.append(String.format("%.2f", Math.abs(coefficients[i]))).append("*distance");
            } else {
                sb.append(String.format("%.2f", Math.abs(coefficients[i]))).append("*distance^").append(i);
            }
            
            first = false;
        }
        
        sb.append("]");
        return sb.toString();
    }
}

