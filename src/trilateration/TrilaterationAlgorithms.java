package trilateration;

import java.io.BufferedReader;
import java.io.IOException;
import java.io.InputStreamReader;
import java.util.ArrayList;
import java.util.Comparator;
import java.util.List;
import java.util.Random;
import java.util.Set;
import java.util.TreeSet;
import java.util.concurrent.ThreadLocalRandom;
import java.util.stream.Collectors;

class Point {
    private int id;
    private double x;
    private double y;
    private double xpr;
    private double ypr;
    private double error;
    private boolean anchor;
    private int cost;
    private Set<Anchor> anchorsInRange;

    public Point(int id, double x, double y) {
        this.id = id;
        this.x=x;
        this.y=y;
        this.xpr = 0;
        this.ypr = 0;
        this.error = 0;
        this.anchor = false;
        this.cost = 0;
        this.anchorsInRange = new TreeSet<>(new Comparator<Anchor>() {
            @Override
            public int compare(Anchor awd1, Anchor awd2) {
                if(awd1.getDistance()>awd2.getDistance()) return 1;
                if(awd1.getDistance()<awd2.getDistance()) return -1;
                else return 0;
            }
        });
    }

    public double getX() {
        return x;
    }

    public double getY() {
        return y;
    }

    public void setXpr(double xpr) {
        this.xpr = xpr;
    }

    public void setYpr(double ypr) {
        this.ypr = ypr;
    }

    public void setError(double error) {
        this.error = error;
    }

    public void setAnchor(boolean anchor) {
        this.anchor = anchor;
    }

    public Set<Anchor> getAnchorsInRange() {
        return anchorsInRange;
    }

    public boolean isAnchor() {
        return anchor;
    }

    public int getCost() {
        return cost;
    }

    public void setCost(int cost) {
        this.cost = cost;
    }
}

class Anchor {
    private Point p;
    private double distance;

    public Anchor(Point p, double distance) {
        this.p=p;
        this.distance=distance;
    }

    public Point getP() {
        return p;
    }

    public double getDistance() {
        return distance;
    }
}

public class TrilaterationAlgorithms {
    private int N, L, R, r, f;
    private List<Point> points;

    public TrilaterationAlgorithms(int N, int L, int R, int r, int f) {
        this.N = N;
        this.L = L;
        this.R = R;
        this.f = f;
        this.r = r;
        this.points = new ArrayList<>();
    }

    public double getNonAnchorsSize() { return (N-(N*(f/100.0))); }

    public double getAnchorsSize() { return (N*(f/100.0)); }

    public double calculateDistance(double x1, double x2, double y1, double y2) {
        return  Math.sqrt(Math.pow((x1-x2), 2) + Math.pow((y1-y2), 2));
    }

    public double gaussRandomValueInRange(double distance) {
        Random random = new Random();
        return (distance-(distance*(r/100.0))) + ((distance+(distance*(r/100.0))) - (distance-(distance*(r/100.0)))) * random.nextDouble();

    }

    public void generateFieldAndPoints() {
        for(int i = 0; i < N; i++) {
            double x = ThreadLocalRandom.current().nextDouble(0, L+1);
            double y = ThreadLocalRandom.current().nextDouble(0, L+1);
            Point p = new Point(i+1, x, y);
            if(i >= getNonAnchorsSize()) {
                p.setAnchor(true);
            }
            points.add(p);
        }

        for(int i = 0; i < getNonAnchorsSize(); i++) {
            for(int j = 0; j < getAnchorsSize(); j++) {
                Point nonAnchor = points.get(i);
                Point anchor = points.get(points.size()-1-j);
                double distance = calculateDistance(nonAnchor.getX(), anchor.getX(), nonAnchor.getY(), anchor.getY());
                double distanceWithError = gaussRandomValueInRange(distance);
                if(distanceWithError <= R) {
                    nonAnchor.getAnchorsInRange().add(new Anchor(anchor, distanceWithError));
                }
            }
        }
    }

    public void nonIterative() {
        float ALE = 0;
        int count = 0;
        for(int i = 0; i < points.size(); i++) {
            if(points.get(i).getAnchorsInRange().size() >= 3) {
                double error = calculateError(i, points);
                ALE += error;
                count++;
            }
        }
        System.out.println("Non-iterative: ALE: " + ALE/count + ", Localized nodes(%): " + (count*100)/getNonAnchorsSize() + "%");
    }

    public void iterativeMostRelevant() {
        List<Point> points_cloned = new ArrayList<>(points);
        int count = 0;
        float ALE = 0;
        for(int i = 0 ; i<points_cloned.size();i++) {
            if(points_cloned.get(i).getAnchorsInRange().size()>=3 && !points_cloned.get(i).isAnchor()) {
                List<Anchor> awd = points_cloned.get(i).getAnchorsInRange().stream().sorted(Comparator.comparing((Anchor a) -> a.getP().getCost()).thenComparing(Anchor::getDistance)).limit(3).collect(Collectors.toList());
                Anchor awd1 = awd.get(0);
                Anchor awd2 = awd.get(1);
                Anchor awd3 = awd.get(2);
                List<Double> newCoordinates = Trilateration(awd1,awd2,awd3);
                points_cloned.get(i).setXpr(newCoordinates.get(0));
                points_cloned.get(i).setYpr(newCoordinates.get(1));
                double error = calculateDistance(points_cloned.get(i).getX(), newCoordinates.get(0), points_cloned.get(i).getY(), newCoordinates.get(1));
                points_cloned.get(i).setError(error);
                points_cloned.get(i).setCost(awd1.getP().getCost() + awd2.getP().getCost() + awd3.getP().getCost() + 1);
                points_cloned.get(i).setAnchor(true);
                for(int j=0;j<points_cloned.size();j++) {
                    if(!points_cloned.get(j).isAnchor() && i!=j) {
                        double distance = calculateDistance(points_cloned.get(j).getX(), points_cloned.get(i).getX(), points_cloned.get(j).getY(), points_cloned.get(i).getY());
                        double distanceWithError = gaussRandomValueInRange(distance);
                        if(distanceWithError <= R) {
                            points_cloned.get(j).getAnchorsInRange().add(new Anchor(points_cloned.get(i), distanceWithError));
                        }
                    }
                }
                i = 0;
                count++;
                ALE += error;
            }
        }
        System.out.println("Iterative three most relevant: ALE: " + ALE/count + ", Localized nodes(%): " + (count*100)/getNonAnchorsSize() + "%");

    }

    public void iterativeThreeClosest() {
        List<Point> points_cloned = new ArrayList<>(points);
        int count = 0;
        float ALE = 0;
        for(int i = 0 ; i<points_cloned.size();i++) {
            if(points_cloned.get(i).getAnchorsInRange().size()>=3 && !points_cloned.get(i).isAnchor()) {
                double error = calculateError(i, points_cloned);
                points_cloned.get(i).setAnchor(true);
                for(int j=0;j<points_cloned.size();j++) {
                    if(!points_cloned.get(j).isAnchor() && i!=j) {
                        double distance = calculateDistance(points_cloned.get(j).getX(), points_cloned.get(i).getX(), points_cloned.get(j).getY(), points_cloned.get(i).getY());
                        double distanceWithError = gaussRandomValueInRange(distance);
                        if(distanceWithError<=R) {
                            points_cloned.get(j).getAnchorsInRange().add(new Anchor(points_cloned.get(i), distanceWithError));
                        }
                    }
                }
                i=0;
                count++;
                ALE += error;
            }
        }
        System.out.println("Iterative three closest: ALE: " + ALE/count + ", Localized nodes(%): " + (count*100)/getNonAnchorsSize() + "%");
    }

    private double calculateError(int i, List<Point> points) {
        List<Anchor> awd = points.get(i).getAnchorsInRange().stream().limit(3).collect(Collectors.toList());
        Anchor awd1 = awd.get(0);
        Anchor awd2 = awd.get(1);
        Anchor awd3 = awd.get(2);
        List<Double> newCoordinates = Trilateration(awd1,awd2,awd3);
        points.get(i).setXpr(newCoordinates.get(0));
        points.get(i).setYpr(newCoordinates.get(1));
        double error = calculateDistance(points.get(i).getX(), newCoordinates.get(0), points.get(i).getY(), newCoordinates.get(1));
        points.get(i).setError(error);
        return error;
    }

    public List<Double> Trilateration(Anchor a1, Anchor a2, Anchor a3) {
        List<Double> newCoordinates = new ArrayList<>();
        double i1 = a1.getP().getX(), i2 = a2.getP().getX(),i3 = a3.getP().getX();
        double j1 = a1.getP().getY(), j2 = a2.getP().getY(),j3 = a3.getP().getY();
        double x, y;
        double d1 = a1.getDistance();
        double d2 = a2.getDistance();
        double d3 = a3.getDistance();

        x = (((((2 * j3) - (2 * j2)) * (((d1 * d1) - (d2 * d2)) + ((i2 * i2) - (i1 * i1)) + ((j2 * j2) - (j1 * j1)))) - (((2 * j2) - (2 * j1)) * (((d2 * d2) - (d3 * d3)) + ((i3 * i3) - (i2 * i2)) + ((j3 * j3) - (j2 * j2))))) / ((((2 * i2) - (2 * i3)) * ((2 * j2) - (2 * j1))) - (((2 * i1) - (2 * i2)) * ((2 * j3) - (2 * j2)))));
        y = (((d1 * d1) - (d2 * d2)) + ((i2 * i2) - (i1 * i1)) + ((j2 * j2) - (j1 * j1)) + (x * ((2 * i1) - (2 * i2)))) / ((2 * j2) - (2 * j1));

        newCoordinates.add(x);
        newCoordinates.add(y);
        return newCoordinates;
    }

    public static void main(String[]args) throws NumberFormatException, IOException {
        BufferedReader br = new BufferedReader(new InputStreamReader(System.in));
        System.out.println("Enter the number of anchors: ");
        int anchorsNum = Integer.parseInt(br.readLine());
        System.out.println("Enter field size: ");
        int fieldSize = Integer.parseInt(br.readLine());
        System.out.println("Enter the radius range of anchor nodes:");
        int radiusRange = Integer.parseInt(br.readLine());
        System.out.println("Enter chance of error in percentage:");
        int errorChancePercentage = Integer.parseInt(br.readLine());
        System.out.println("Enter anchor fraction:");
        int anchorFraction = Integer.parseInt(br.readLine());

        TrilaterationAlgorithms f = new TrilaterationAlgorithms(anchorsNum, fieldSize, radiusRange, errorChancePercentage, anchorFraction);
        f.generateFieldAndPoints();
        f.nonIterative();
		f.iterativeMostRelevant();
        f.iterativeThreeClosest();
    }
}