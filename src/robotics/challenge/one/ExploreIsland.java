package robotics.challenge.one;

import lejos.robotics.subsumption.Arbitrator;
import lejos.robotics.subsumption.Behavior;

public class ExploreIsland {

	public static void main(String[] args) {
	Behavior findLine = new Behavior() {
		
			@Override
			public boolean takeControl() {
				// TODO Auto-generated method stub
				return false;
			}
			
			@Override
			public void suppress() {
				//suppressed = true;
				//should higher behaviors call suppress?
				
			}
			
			@Override
			public void action() {
				//Drive around randomly
				
			}
		};
		
		Behavior followLine = new Behavior() {
			
			@Override
			public boolean takeControl() {
				//takeControl if on white line
				return false;
			}
			
			@Override
			public void suppress() {
				//suppressed = true;
				
			}
			
			@Override
			public void action() {
				//follow the white line
				
			}
		};
		
		Behavior corner = new Behavior() {
			
			@Override
			public boolean takeControl() {
				//takeControl if on corner
				return false;
			}
			
			@Override
			public void suppress() {
				//suppressed = true
				
			}
			
			@Override
			public void action() {
				//make a sound
				
			}
		};
		
		Behavior intersection = new Behavior() {
			
			@Override
			public boolean takeControl() {
				//takeControl if on intersection
				return false;
			}
			
			@Override
			public void suppress() {
				//suppressed = true
				
			}
			
			@Override
			public void action() {
				//make a sound
				
			}
		};
		
		Behavior bluePillar = new Behavior() {
			
			@Override
			public boolean takeControl() {
				//takeControl if sees blue pillar
				return false;
			}
			
			@Override
			public void suppress() {
				//suppressed = true
				
			}
			
			@Override
			public void action() {
				//remember the location of the blue pillar 
				
			}
		};
		
		Behavior [] bArray = {findLine,followLine,corner,intersection,bluePillar};
		
		Arbitrator arby = new Arbitrator(bArray);
		
		arby.start();
	}

}
