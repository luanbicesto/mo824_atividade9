package gurobi;

public class Instance {
    private int size;
    private double[][] edgeCosts;
    private int[][] nodesPosition;
    private double[] customerDemands;
    private double vehicleCapacity;

    public Instance() {
    }
    
    public int getSize() {
        return size;
    }

    public void setSize(int size) {
        this.size = size;
    }

    public double[][] getEdgeCosts() {
        return edgeCosts;
    }

    public void setEdgeCosts(double[][] edgeCosts) {
        this.edgeCosts = edgeCosts;
    }

    public double[] getCustomerDemands() {
        return customerDemands;
    }

    public void setCustomerDemands(double[] customerDemands) {
        this.customerDemands = customerDemands;
    }

    public double getVehicleCapacity() {
        return vehicleCapacity;
    }

    public void setVehicleCapacity(double vehicleCapacity) {
        this.vehicleCapacity = vehicleCapacity;
    }

    public int[][] getNodesPosition() {
        return nodesPosition;
    }

    public void setNodesPosition(int[][] nodesPosition) {
        this.nodesPosition = nodesPosition;
    }
    
    public void computeEdgesCost() {
        setEdgeCosts(new double[getSize()][getSize()]);
        for(int i = 0; i < getSize(); i++) {
            for(int j = 0; j < getSize(); j++) {
                getEdgeCosts()[i][j] = computeDistanceVertices(i, j);
            }
        }
    }
    
    private double computeDistanceVertices(int v1, int v2) {
        double distance = 0.0;
        
        int[] positionV1 = getNodesPosition()[v1];
        int[] positionV2 = getNodesPosition()[v2];
        
        distance = Math.pow(positionV1[0] - positionV2[0], 2) + Math.pow(positionV1[1] - positionV2[1], 2);
        
        return Math.round(Math.sqrt(distance));
    }
    
}
