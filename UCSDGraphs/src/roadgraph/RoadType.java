package roadgraph;
import java.util.*;

public class RoadType {

	private int speedLimit;
	private String type;
	
	RoadType(String type) {
		this.type = type;
	}
	
	void setSpeedLimit() {
	if(type.equals("residential")) {
		speedLimit = 20000;
	}
	else if(type.equals("tertiary") || type.equals("motorway_link")) {
		speedLimit = 30000;
	}
	else if(type.equals("secondary")) {
		speedLimit = 40000;
	}
	else if(type.equals("primary")) {
		speedLimit = 60000;
	}
	else if(type.equals("city street")) {
		speedLimit = 35000;
	}
	else 
		speedLimit = 45000;
	}

   int getSpeedLimit() {
	   return speedLimit;
   }
}