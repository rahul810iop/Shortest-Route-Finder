package roadgraph;

import java.util.*;
import geography.GeographicPoint;

public class MapEdge {
    /** two variables for each edge*/
	private MapNode start;
	private MapNode end;
	/**length of each edge*/
	private double length;
	/** to store edge details*/ 
	private String roadName,roadType;
	
	private double duration;
	
	static final double DEFAULT_LENGTH = 0.01;
	
	MapEdge(MapNode start, MapNode end,String roadName, String roadType)
	{
		this.start = start;
		this.end = end;
		this.roadName = roadName;
		this.roadType = roadType;
		this.length = DEFAULT_LENGTH;
		
	}
	
	MapEdge(MapNode start, MapNode end,String roadName, String roadType, double length)
	{
		this.start = start;
		this.end = end;
		this.roadName = roadName;
		this.roadType = roadType;
		this.length = length;
	}
	
	// return start point
    MapNode getStartNode() {
    	return start;
    }
    //return end point
    MapNode getEndNode() {
    	return end;
    }
    //return the roadName
    public String getRoadName() {
    	return roadName;
    }
    //return the roadType
    public String getRoadType() {
    	return roadType;
    }
    //return the Edge Length
    double getLength() {
    	return length;
    }
    
    void setDuration(String roadType) {
    	RoadType rd = new RoadType(roadType);
    	duration = (length/rd.getSpeedLimit());
    }
    
    double getDuration() {
    	return duration;
    }
    //to return the other node of the edge
    public MapNode getOtherNode(MapNode node) {
    	if(node.equals(start))
    		return end;
    	else if(node.equals(end))
    		return start;
    	throw new IllegalArgumentException("Point you are looking may not be present int the edge");
    }
}
