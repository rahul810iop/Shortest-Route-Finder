/**
 * @author UCSD MOOC development team and YOU
 * 
 * A class which reprsents a graph of geographic locations
 * Nodes in the graph are intersections between 
 *
 */
package roadgraph;

import java.util.*;
import java.util.function.Consumer;

import geography.GeographicPoint;
import util.GraphLoader;
/**
 * @author UCSD MOOC development team and YOU
 * 
 * A class which represents a graph of geographic locations
 * Nodes in the graph are intersections between 
 *
 */
public class MapGraph  {
	//TODO: Add your member variables here in WEEK 3
	
	HashMap<GeographicPoint,MapNode> nodeMap;
	HashSet<MapEdge> edges;
	/** 
	 * Create a new empty MapGraph 
	 */
	public MapGraph()
	{
		// TODO: Implement in this constructor in WEEK 3
		nodeMap = new HashMap<GeographicPoint,MapNode> ();
		edges = new HashSet<MapEdge> ();
	}
	
	/**
	 * Get the number of vertices (road intersections) in the graph
	 * @return The number of vertices in the graph.
	 */
	public int getNumVertices()
	{
		//TODO: Implement this method in WEEK 3
		return nodeMap.size();
	}
	
	/**
	 * Return the intersections, which are the vertices in this graph.
	 * @return The vertices in this graph as GeographicPoints
	 */
	public Set<GeographicPoint> getVertices()
	{
		//TODO: Implement this method in WEEK 3
		Set<GeographicPoint> nodeList = nodeMap.keySet();
		return nodeList;
	}
	
	/**
	 * Get the number of road segments in the graph
	 * @return The number of edges in the graph.
	 */
	public int getNumEdges()
	{
		//TODO: Implement this method in WEEK 3
		return edges.size();
	}

	
	/** Add a node corresponding to an intersection at a Geographic Point
	 * If the location is already in the graph or null, this method does 
	 * not change the graph.
	 * @param location  The location of the intersection
	 * @return true if a node was added, false if it was not (the node
	 * was already in the graph, or the parameter is null).
	 */
	public boolean addVertex(GeographicPoint location)
	{
		// TODO: Implement this method in WEEK 3
		if(location == null)
			return false;
		
		MapNode node = nodeMap.get(location);
		if (node == null) {
			node = new MapNode(location);
			nodeMap.put(location, node);
			return true;
		}
		else {
			System.out.println("Warning: Node at location " + location + " already exists in the graph.");
			return false;
		}
	}
	
	/**
	 * Adds a directed edge to the graph from pt1 to pt2.  
	 * Precondition: Both GeographicPoints have already been added to the graph
	 * @param from The starting point of the edge
	 * @param to The ending point of the edge
	 * @param roadName The name of the road
	 * @param roadType The type of the road
	 * @param length The length of the road, in km
	 * @throws IllegalArgumentException If the points have not already been
	 *   added as nodes to the graph, if any of the arguments is null,
	 *   or if the length is less than 0.
	 */
	public void addEdge(GeographicPoint from, GeographicPoint to, String roadName,
			String roadType, double length) throws IllegalArgumentException {

		//TODO: Implement this method in WEEK 3
		MapNode start = nodeMap.get(from);
		MapNode end = nodeMap.get(to);
		
			if (start == null)
				throw new NullPointerException("from: " + from + "does not exist");
			if (end == null)
				throw new NullPointerException("to: " + to + "does not exist");

		  MapEdge edge = new MapEdge(start, end, roadName, roadType, length);
		    edges.add(edge);
			start.addEdge(edge);
	}
	  

	/** Find the path from start to goal using breadth first search
	 * 
	 * @param start The starting location
	 * @param goal The goal location
	 * @return The list of intersections that form the shortest (unweighted)
	 *   path from start to goal (including both start and goal).
	 */
	public List<GeographicPoint> bfs(GeographicPoint start, GeographicPoint goal) {
		// Dummy variable for calling the search algorithms
        Consumer<GeographicPoint> temp = (x) -> {};
        return bfs(start, goal, temp);
	}
	
	/** Find the path from start to goal using breadth first search
	 * 
	 * @param start The starting location
	 * @param goal The goal location
	 * @param nodeSearched A hook for visualization.  See assignment instructions for how to use it.
	 * @return The list of intersections that form the shortest (unweighted)
	 *   path from start to goal (including both start and goal).
	 */
	public List<GeographicPoint> bfs(GeographicPoint start, 
			 					     GeographicPoint goal, Consumer<GeographicPoint> nodeSearched)
	{
		// TODO: Implement this method in WEEK 3
		
		// Hook for visualization.  See writeup.
		//nodeSearched.accept(next.getLocation());
		
        MapNode startingNode = nodeMap.get(start);
        MapNode goalNode = nodeMap.get(goal);
        
        if (start == null || goal == null)
			throw new NullPointerException("Cannot find route from or to null node");
		if (startingNode == null || goalNode == null) {
			System.out.println("Start or goal node is null! No path exists.");
			return null;
		}
		if(start==goal)
		{
			System.out.println("Start and end nodes are the same");
			return null;
		}
		HashMap<MapNode, MapNode> parentMap = new HashMap<MapNode, MapNode>();
		boolean found = bfsSearch(startingNode, goalNode, parentMap, nodeSearched);
		
		if (!found) {
			System.out.println("No path found from " + start + " to " + goal);
			return null;
		}
		
		// Reconstruct the parent path
		List<GeographicPoint> path = constructPath(startingNode, goalNode, parentMap);

		return path;
	}
	
	private boolean bfsSearch(MapNode startingNode, MapNode goalNode,HashMap<MapNode,MapNode> parentMap, Consumer<GeographicPoint> nodeSearched) {
		
        Queue<MapNode> toExplore = new LinkedList<MapNode>();
        Set<MapNode> visited = new HashSet<MapNode>();
        boolean found = false;
        toExplore.add(startingNode);
        visited.add(startingNode);
        
        while(!toExplore.isEmpty()) {
        	MapNode curr = toExplore.remove();
        	
        	nodeSearched.accept(curr.getLocation());
        	// Hook for visualization.  See writeup.
        	if(curr == goalNode) {
        	  found = true;
        	  break;
        	}
        	Set<MapNode> neighbors = curr.getNeighbors();
			for (MapNode next : neighbors) {
				if (!visited.contains(next)) {
					visited.add(next);
					parentMap.put(next, curr);
					toExplore.add(next);
				}
			}	
        }
		return found;
	}
	
	/** Find the path from start to goal using breadth first search
	 * 
	 * @param start The starting location
	 * @param goal The goal location
	 * @return The list of intersections that form the shortest (unweighted)
	 *   path from start to goal (including both start and goal).
	 */
	
    public List<GeographicPoint> constructPath(MapNode start, MapNode goal,HashMap<MapNode,MapNode> parentMap) {
    	LinkedList<GeographicPoint> path = new LinkedList<GeographicPoint>();
		MapNode curr = goal;
		
		while (!curr.equals(start)) {
			path.addFirst(curr.getLocation());
			curr = parentMap.get(curr);
		}
		
		// add start
		path.addFirst(start.getLocation());
		return path;

	}	

	/** Find the path from start to goal using Dijkstra's algorithm
	 * 
	 * @param start The starting location
	 * @param goal The goal location
	 * @return The list of intersections that form the shortest path from 
	 *   start to goal (including both start and goal).
	 */
	public List<GeographicPoint> dijkstra(GeographicPoint start, GeographicPoint goal) {
		// Dummy variable for calling the search algorithms
		// You do not need to change this method.
        Consumer<GeographicPoint> temp = (x) -> {};
        return dijkstra(start, goal, temp);
	}
	
	/** Find the path from start to goal using Dijkstra's algorithm
	 * 
	 * @param start The starting location
	 * @param goal The goal location
	 * @param nodeSearched A hook for visualization.  See assignment instructions for how to use it.
	 * @return The list of intersections that form the shortest path from 
	 *   start to goal (including both start and goal).
	 */
	public List<GeographicPoint> dijkstra(GeographicPoint start, 
										  GeographicPoint goal, Consumer<GeographicPoint> nodeSearched)
	{
		// TODO: Implement this method in WEEK 4
		
		if (start == null || goal == null)
			throw new NullPointerException("Cannot find route from or to null node");
		if(start==goal)
		{
			System.out.println("Start and end nodes are the same");
			return null;
		}
		
		MapNode startNode = nodeMap.get(start);
		MapNode endNode = nodeMap.get(goal);
		if (startNode == null || endNode == null) {
			System.out.println("Start or goal node is null! No path exists.");
			return null;
		}
		
        PriorityQueue<MapNode> toExplore = new PriorityQueue<MapNode>(); 
		HashSet<MapNode> visited = new HashSet<MapNode>();
        HashMap<MapNode,MapNode> parentMap = new HashMap<MapNode,MapNode>();
        
        for(MapNode node : nodeMap.values()) {
             node.setCurrDistance(Double.POSITIVE_INFINITY);
        }
        
        
        boolean found = dijkstraSearch(startNode, endNode, parentMap, toExplore, visited, nodeSearched);
		if (!found) {
			System.out.println("No path found from " + start + " to " + goal);
			return null;
		}
		
		// Reconstruct the parent path
		List<GeographicPoint> path = constructPath(startNode, endNode, parentMap);

		return path;
        
	}

	private boolean dijkstraSearch(MapNode start, 
			  MapNode goal,HashMap<MapNode,MapNode> parentMap,
			  PriorityQueue<MapNode> toExplore, HashSet<MapNode> visited, Consumer<GeographicPoint> nodeSearched)
       {
          // TODO: Implement this method in WEEK 4    
         
          boolean found = false;
          start.setCurrDistance(0.0);
          int nodeCount = 0;
          
          toExplore.add(start);
          
          while(!toExplore.isEmpty()) {
        	  
             MapNode curr = toExplore.remove();
             nodeCount++;
          // Hook for visualization.  See writeup.
 			nodeSearched.accept(curr.getLocation());
 			
 			System.out.println("Dijkstra visiting " + curr.getLocation());
 			if(curr == goal) {
       		 found = true;
       		 System.out.println("Nodes Visited: "+nodeCount);
       		 break;
 			}
             if(!visited.contains(curr)) {
            	 visited.add(curr);
            	 
            	 Set<MapEdge> associatedEdges = curr.getAssociatedEdges();
     			 for (MapEdge next : associatedEdges) {
     				 MapNode neighbor = next.getEndNode();
     				 if(!visited.contains(neighbor)) {
     					double currDist = next.getLength() + curr.getCurrDistance();
						if (currDist < neighbor.getCurrDistance()) {
							parentMap.put(neighbor, curr);
							neighbor.setCurrDistance(currDist);
							toExplore.add(neighbor);
						} 
						
     				 }
     				 
     			   }
     			 
                 }
             }

         return found;
      } 
	
	/*public List<GeographicPoint> dijkstraUsingDuration(GeographicPoint start, GeographicPoint goal) {
		// Dummy variable for calling the search algorithms
		// You do not need to change this method.
        Consumer<GeographicPoint> temp = (x) -> {};
        return dijkstra(start, goal, temp);
	}
	
	public List<GeographicPoint> dijkstraUsingDuration(GeographicPoint start, 
			  GeographicPoint goal, Consumer<GeographicPoint> nodeSearched)
{


            if (start == null || goal == null)  
                throw new NullPointerException("Cannot find route from or to null node");
                if(start==goal)
                  {
                    System.out.println("Start and end nodes are the same");
                    return null;
                  }

        MapNode startNode = nodeMap.get(start);
        MapNode endNode = nodeMap.get(goal);
        if (startNode == null || endNode == null) {
            System.out.println("Start or goal node is null! No path exists.");
            return null;
        }

        PriorityQueue<MapNode> toExplore = new PriorityQueue<MapNode>(); 
        HashSet<MapNode> visited = new HashSet<MapNode>();
        HashMap<MapNode,MapNode> parentMap = new HashMap<MapNode,MapNode>();

        for(MapNode node : nodeMap.values()) {
             node.setCurrDistance(Double.POSITIVE_INFINITY);
        }


        boolean found = dijkstraSearchUsingDuration(startNode, endNode, parentMap, toExplore, visited, nodeSearched);
        if (!found) {
             System.out.println("No path found from " + start + " to " + goal);
          return null;
       }

// Reconstruct the parent path
List<GeographicPoint> path = constructPath(startNode, endNode, parentMap);

return path;

}

      private boolean dijkstraSearchUsingDuration(MapNode start, 
                              MapNode goal,HashMap<MapNode,MapNode> parentMap,PriorityQueue<MapNode> toExplore, HashSet<MapNode> visited, Consumer<GeographicPoint> nodeSearched)
       {
    

             boolean found = false;
             start.setCurrDuration(0.0);
             int nodeCount = 0;

             toExplore.add(start);

             while(!toExplore.isEmpty()) {

                  MapNode curr = toExplore.remove();
                  nodeCount++;
// Hook for visualization.  See writeup.
                  nodeSearched.accept(curr.getLocation());

                  System.out.println("Dijkstra visiting " + curr.getLocation());
                  if(curr == goal) {
                      found = true;
                      System.out.println("Nodes Visited: "+nodeCount);
                      break;
                  }
                  if(!visited.contains(curr)) {
                       visited.add(curr);

                    Set<MapEdge> associatedEdges = curr.getAssociatedEdges();
                       for (MapEdge next : associatedEdges) {
                               MapNode neighbor = next.getEndNode();
                                if(!visited.contains(neighbor)) {
                                    double currDur = next.getDuration() + curr.getCurrDuration();
                                   if (currDur < neighbor.getCurrDuration()) {
                                       parentMap.put(neighbor, curr);
                                       neighbor.setCurrDistance(currDur);
                                       toExplore.add(neighbor);
                                    } 

                                }

                           }

                     }
            }

         return found;
     } */



	/** Find the path from start to goal using A-Star search
	 * 
	 * @param start The starting location
	 * @param goal The goal location
	 * @return The list of intersections that form the shortest path from 
	 *   start to goal (including both start and goal).
	 */
	public List<GeographicPoint> aStarSearch(GeographicPoint start, GeographicPoint goal) {
		// Dummy variable for calling the search algorithms
        Consumer<GeographicPoint> temp = (x) -> {};
        return aStarSearch(start, goal, temp);
	}
	
	/** Find the path from start to goal using A-Star search
	 * 
	 * @param start The starting location
	 * @param goal The goal location
	 * @param nodeSearched A hook for visualization.  See assignment instructions for how to use it.
	 * @return The list of intersections that form the shortest path from 
	 *   start to goal (including both start and goal).
	 */
	public List<GeographicPoint> aStarSearch(GeographicPoint start, 
											 GeographicPoint goal, Consumer<GeographicPoint> nodeSearched)
	{
		// TODO: Implement this method in WEEK 4
		
		// Hook for visualization.  See writeup.
		//nodeSearched.accept(next.getLocation());
		if (start == null || goal == null)
			throw new NullPointerException("Cannot find route from or to null node");
		if(start==goal)
		{
			System.out.println("Start and end nodes are the same");
			return null;
		}
		
		MapNode startNode = nodeMap.get(start);
		MapNode endNode = nodeMap.get(goal);
		if (startNode == null || endNode == null) {
			System.out.println("Start or goal node is null! No path exists.");
			return null;
		}
		
        PriorityQueue<MapNode> toExplore = new PriorityQueue<MapNode>(); 
		HashSet<MapNode> visited = new HashSet<MapNode>();
        HashMap<MapNode,MapNode> parentMap = new HashMap<MapNode,MapNode>();
        
        for(MapNode node : nodeMap.values()) {
             node.setCurrDistance(Double.POSITIVE_INFINITY);
        }
         
        boolean found = aStarSearchAlgo(startNode, endNode, parentMap, toExplore, visited, nodeSearched);
        
		if (!found) {
			System.out.println("No path found from " + start + " to " + goal);
			return null;
		}
		
		List<GeographicPoint> path = constructPath(startNode, endNode, parentMap);

		return path;

	}

	private static boolean aStarSearchAlgo(MapNode startNode, MapNode endNode, HashMap<MapNode, MapNode> parentMap, PriorityQueue<MapNode> toExplore, 
			HashSet<MapNode> visited, Consumer<GeographicPoint> nodeSearched) {
		startNode.setCurrDistance(0);
		startNode.setActualDistance(0);

		toExplore.add(startNode);
		
		int nodeCount = 0;
		boolean found = false;
		
		while (!toExplore.isEmpty()) {
			MapNode curr = toExplore.remove();
            nodeCount++;
            
			// Hook for visualization.  See writeup.
            nodeSearched.accept(curr.getLocation());

            // debug
			System.out.println("\nA* visiting " + curr.getLocation() +"\nActual = "+ curr.getActualDistance()+", Pred: "+ curr.getCurrDistance());
			if (curr.equals(endNode)) {
				found = true;
				System.out.println("Nodes visited in search: "+nodeCount);
				break;
			}
			if(!visited.contains(curr)) {
				visited.add(curr);
				Set<MapEdge> edges = curr.getAssociatedEdges();
				for (MapEdge edge : edges) {
					MapNode neighbor = edge.getEndNode();
					if (!visited.contains(neighbor)) {

						double currDist = edge.getLength()+curr.getActualDistance();
						// core of A* is just to add to currDist the cost of getting to
						// the destination
						double predDist = currDist+ (neighbor.getLocation()).distance(endNode.getLocation());
						if(predDist < neighbor.getCurrDistance()){
							// debug
							// System.out.println("Adding to queue node at: "+neighbor.getLocation());
							// System.out.println("Curr dist: "+currDist+" Pred Distance: " + predDist);
							
							parentMap.put(neighbor, curr);
							neighbor.setActualDistance(currDist);
							neighbor.setCurrDistance(predDist);
							toExplore.add(neighbor);
						}
					}
				}
			}
		}
		return found;
	}
		
	public static void main(String[] args)
	{
		/*System.out.print("Making a new map...");
		MapGraph firstMap = new MapGraph();
		System.out.print("DONE. \nLoading the map...");
		GraphLoader.loadRoadMap("data/testdata/simpletest.map", firstMap);
		System.out.println("DONE.");
		
		// You can use this method for testing.  
		List<GeographicPoint> route = firstMap.bfs(new GeographicPoint(1.0, 1.0), new GeographicPoint(8.0, -1.0));
		System.out.println(route);*/
		
		/* Here are some test cases you should try before you attempt 
		 * the Week 3 End of Week Quiz, EVEN IF you score 100% on the 
		 * programming assignment.
		 */
		
		/*MapGraph simpleTestMap = new MapGraph();
		GraphLoader.loadRoadMap("data/testdata/simpletest.map", simpleTestMap);
		
		GeographicPoint testStart = new GeographicPoint(1.0, 1.0);
		GeographicPoint testEnd = new GeographicPoint(8.0, -1.0);
		
		System.out.println("Test 1 using simpletest: Dijkstra should be 9 and AStar should be 5");
		List<GeographicPoint> testroute = simpleTestMap.dijkstra(testStart,testEnd);
		List<GeographicPoint> testroute2 = simpleTestMap.aStarSearch(testStart,testEnd);
		System.out.println("***********************");
		//System.out.println(testroute2);
		
		MapGraph testMap = new MapGraph();
		GraphLoader.loadRoadMap("data/maps/utc.map", testMap);
		
		// A very simple test using real data
		testStart = new GeographicPoint(32.869423, -117.220917);
		testEnd = new GeographicPoint(32.869255, -117.216927);
		System.out.println("Test 2 using utc: Dijkstra should be 13 and AStar should be 5");
		testroute = testMap.dijkstra(testStart,testEnd);
		testroute2 = testMap.aStarSearch(testStart,testEnd);
		System.out.println("************************");
		
		// A slightly more complex test using real data
		testStart = new GeographicPoint(32.8674388, -117.2190213);
		testEnd = new GeographicPoint(32.8697828, -117.2244506);
		System.out.println("Test 3 using utc: Dijkstra should be 37 and AStar should be 10");
		testroute = testMap.dijkstra(testStart,testEnd);
		testroute2 = testMap.aStarSearch(testStart,testEnd);*/
		
		
		/* Use this code in Week 3 End of Week Quiz */
		MapGraph theMap = new MapGraph();
		System.out.print("DONE. \nLoading the map...");
		GraphLoader.loadRoadMap("data/maps/utc.map", theMap);
		System.out.println("DONE.");

		GeographicPoint start = new GeographicPoint(32.8648772, -117.2254046);
		GeographicPoint end = new GeographicPoint(32.8660691, -117.217393);
		
		
		List<GeographicPoint> route = theMap.dijkstra(start,end);
		List<GeographicPoint> route2 = theMap.aStarSearch(start,end);

		
		
	}
	
}
