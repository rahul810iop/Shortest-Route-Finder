package roadgraph;

import java.util.*;
import geography.GeographicPoint;

public class MapNode implements Comparable{
	/** The list of edges out of this node */
	private HashSet<MapEdge> edges;
	
	/** the latitude and longitude of this node */
	private GeographicPoint location;
	
	/** initialize the fields */
	private double currDistance;
	
	/**Direct Distance(as crow flies)*/
	private double actualDistance;
	
	private double currDuration;
	
	MapNode(GeographicPoint location) {
		this.location = location;
		edges = new HashSet<MapEdge>();
		currDistance = 0.0;
	}
	
	HashSet<MapEdge> getAssociatedEdges() {
		return edges;
	}
	void addEdge(MapEdge edge) {
		edges.add(edge);
	}
	
	GeographicPoint getLocation() {
		return location;
	}
	
	double getCurrDistance() {
		return currDistance;
	}
	
	void setCurrDistance(double value) {
		this.currDistance = value;
	}
	
	double getActualDistance() {
		return actualDistance;
	}
	
	void setActualDistance(double value) {
		this.actualDistance = actualDistance;
	}
	
	void setCurrDuration(double value) {
		currDuration = value;
	}
	
	double getCurrDuration() {
		return currDuration;
	}
	
	public int compareTo(Object obj) {
		MapNode temp = (MapNode)obj;
		if( currDistance > temp.currDistance) {
			return 1;  
	    }else if(currDistance < temp.currDistance){  
	        return -1;  
	    }else{  
	    return 0;
		}
	}
	
	/*public int compareTo(Object obj) {
		MapNode temp = (MapNode)obj;
		if( currDuration > temp.currDuration) {
			return 1;  
	    }else if(currDuration < temp.currDuration){  
	        return -1;  
	    }else{  
	    return 0;
		}
	}*/
	/** to get the neighbors of this MapNode */
	Set<MapNode> getNeighbors() {
		Set<MapNode> neighbors = new HashSet<MapNode>();
		for(MapEdge edge : edges) {
			neighbors.add(edge.getOtherNode(this));
		}
		return neighbors;	
	}
	
}
