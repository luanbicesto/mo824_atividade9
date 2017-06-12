package gurobi;

import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Paths;
import java.util.HashSet;
import java.util.Iterator;
import java.util.Set;
import java.util.StringTokenizer;
import java.util.stream.Stream;

public class GurobiCVRP {
    private Instance instance;
    private GRBVar[][] edges;
    private GRBVar vehicles;
    private GRBEnv env;
    private GRBModel model;
    
    public static void main(String[] args) {
        GurobiCVRP lpiSolver = new GurobiCVRP();
        lpiSolver.solve();
    }
    
    private void createEdgesVariables() throws GRBException {
        edges = new GRBVar[instance.getSize()][instance.getSize()];
        for(int i = 0; i < instance.getSize(); i++) {
            for(int j = i; j < instance.getSize(); j++) {
                edges[i][j] = model.addVar(0.0, 1.0, instance.getEdgeCosts()[i][j],
                        GRB.BINARY,
                      "x"+String.valueOf(i)+"_"+String.valueOf(j));
                
                edges[j][i] = edges[i][j];
            }
        }
    }
    
    private void createVehiclesVariable() throws GRBException {
        vehicles = model.addVar(0, instance.getSize(), 1, GRB.INTEGER, "vehicles");
    }
    
    private void eachVertexDegree2WithoutDepot() throws GRBException {
        GRBLinExpr expr;
        for(int i = 1; i < instance.getSize(); i++) {
            expr = new GRBLinExpr();
            for(int j = 0; j < instance.getSize(); j++) {
                expr.addTerm(1, edges[i][j]);
            }
            model.addConstr(expr, GRB.EQUAL, 2.0, "deg2_"+String.valueOf(i));
        }
    }
    
    private void depotDegree() throws GRBException {
        GRBLinExpr expr = new GRBLinExpr();
        for(int j = 0; j < instance.getSize(); j++) {
            expr.addTerm(1, edges[0][j]);
        }
        GRBLinExpr exprLimitVehicles = new GRBLinExpr();
        exprLimitVehicles.addTerm(2, vehicles);
        model.addConstr(expr, GRB.EQUAL, exprLimitVehicles, "depot_degree");
    }
    
    private void dennyAutoCycle() throws GRBException {
        for (int i = 0; i < instance.getSize(); i++)
            edges[i][i].set(GRB.DoubleAttr.UB, 0.0);
    }
    
    private void readInstance(String name) throws IOException {
        Stream<String> stream = Files.lines(Paths.get("vrp_instances/" + name));
        Iterator<String> iterator = stream.iterator();
        this.instance = new Instance();
        
        this.instance.setSize(Integer.parseInt(iterator.next().toString()));
        this.instance.setVehicleCapacity(Double.parseDouble(iterator.next().toString()));
        
        //read positions
        instance.setNodesPosition(new int[instance.getSize()][2]);
        for(int i = 0; i < instance.getSize(); i++) {
            String position = iterator.next().toString();
            StringTokenizer tokens = new StringTokenizer(position);
            tokens.nextToken();//id vertex
            instance.getNodesPosition()[i][0] = Integer.parseInt(tokens.nextToken());
            instance.getNodesPosition()[i][1] = Integer.parseInt(tokens.nextToken());
        }
        instance.computeEdgesCost();
        
        //read customer demands
        instance.setCustomerDemands(new double[instance.getSize()]);
        for(int i = 0; i < instance.getSize(); i++) {
            String capacity = iterator.next().toString();
            StringTokenizer tokens = new StringTokenizer(capacity);
            tokens.nextToken();//id vertex
            instance.getCustomerDemands()[i] = Double.parseDouble(tokens.nextToken());
        }
    }
    
    public void solve() {
        try{
            readInstance("instance1.vrp");
            env = new GRBEnv();
            model = new GRBModel(env);
            model.set(GRB.DoubleParam.TimeLimit, 1800);//Limite de tempo de execução de 30 minutos
            model.set(GRB.IntParam.LazyConstraints, 1);
            
            createEdgesVariables();
            createVehiclesVariable();
            eachVertexDegree2WithoutDepot();
            depotDegree();
            dennyAutoCycle();
            
            model.setCallback(new TspCallback(edges, instance));
            model.optimize();
        }catch(Exception ex) {
            
        }
    }
    
    private class TspCallback extends GRBCallback {
        private GRBVar[][] edges;
        private Instance instance;
        private static final int DEPOT = 0;
        
        public TspCallback(GRBVar[][] edges, Instance instance) {
            this.edges = edges;
            this.instance = instance;
        }
        
        @Override
        protected void callback() {
            try {
                if (where == GRB.CB_MIPSOL) {
                    int[] tour = findsubtour(getSolution(edges));

                    if(tour != null) {
                        addCut(tour);
                    }
                }
            } catch (GRBException e) {
                System.out.println("Error code: " + e.getErrorCode() + ". " + e.getMessage());
                e.printStackTrace();
            }
        }
        
        private void addCut(int[] tour) throws GRBException {
            Set<Integer> nodesTour = new HashSet<>();
            GRBLinExpr expr = new GRBLinExpr();
            double totalDemand = 0.0;
            
            for(int i = 0; i < tour.length; i++) {
                if(tour[i] != DEPOT) {
                    nodesTour.add(tour[i]);
                    totalDemand += instance.getCustomerDemands()[tour[i]];
                }
            }
            
            for(Integer node : nodesTour) {
                for(int i = 0; i < instance.getSize(); i++) {
                    if(!nodesTour.contains(i)) {
                        expr.addTerm(1.0, edges[node][i]);
                    }
                }
            }
            
            addLazy(expr, GRB.GREATER_EQUAL, totalDemand/instance.getVehicleCapacity());
        }
        
        private int[] findsubtour(double[][] sol) {
            int n = sol.length;
            boolean[] seen = new boolean[n];
            boolean cycleContainsDepot = false;
            int[] tour = new int[n];
            int bestind, bestlen;
            int i, node, len, start;
            double currentTotalDemand = 0.0;

            for (i = 0; i < n; i++)
                seen[i] = false;

            start = 0;
            bestlen = n + 1;
            bestind = -1;
            node = 0;
            
            while (start < n) {
                for (node = 0; node < n; node++)
                    if (!seen[node])
                        break;
                if (node == n)
                    break;
                for (len = 0; len < n; len++) {
                    cycleContainsDepot = node == DEPOT;
                    tour[start + len] = node;
                    currentTotalDemand += instance.getCustomerDemands()[node];
                    seen[node] = true;
                    for (i = 0; i < n; i++) {
                        if (sol[node][i] > 0.5 && !seen[i]) {
                            node = i;
                            break;
                        }
                    }
                    if (i == n) {
                        if(node != DEPOT) {
                            len++;
                            if(!cycleContainsDepot || isDoubleGrater(currentTotalDemand, instance.getVehicleCapacity())) {
                                if (len < bestlen) {
                                    bestlen = len;
                                    bestind = start;
                                }
                            } else {
                                seen[DEPOT] = false;
                            }
                            start += len;
                        }
                        break;
                    }
                }
            }

            int result[] = null;
            if(bestlen != (n+1)) {
                result = new int[bestlen];
                for (i = 0; i < bestlen; i++)
                    result[i] = tour[bestind + i];
            }
            
            return result;
        }
        
        private boolean isDoubleGrater(double d1, double d2) {
            return Double.compare(d1, d2) > 0;
        }
    }

}
