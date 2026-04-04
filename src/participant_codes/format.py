'''
INSTRUCTIONS:
1. Participants must copy this file (without any edits) and make their own file with the team name which has been communicated to the organizers. 
For example, if your team name is "Alpha", you should create a file named "alpha.py" in the same directory as this file.

2. In your team file, edit only the function "mission_plan" to return a list of waypoints for the drone to follow. 
Each waypoint should be a tuple of (x, y, altitude) in meters relative to the takeoff point.

3. Do not edit any other part of the code, and do not modify any other directory or file. 
The organizers will run your code in an environment where the rest of the system is already set up, and any changes to other files may cause your code to break.
Failure to follow these instructions may result in disqualification.

4. The "mission_plan" function should return a list of waypoints that form a valid mission for the drone.
The wapoints should be designed to test the drone's ability to navigate and perform tasks, and should be within the operational limits of the drone.
The drone MUST NOT exceed the limited bounds which will be provided by organizers during the event.

5. The organizers will evaluate the missions based on criteria such as creativity, complexity, and how well they test the drone's capabilities.

6. The software will first be run in a simulation environment, and then on a real drone if the team scores well enough.

7. Feel free to use any resources or libraries you need to design your mission, but remember that the drone's performance will depend on how well the mission is planned.
The waypoints must be sampled properly within the bounds and should not be too close to each other to avoid collisions or too far to be unfeasible.

8. The organizers will provide feedback on the missions, and teams will have the opportunity to refine their missions based on this feedback.

9. The final submission should include the team file with the "mission_plan" function implemented, and any additional documentation or explanations about the mission design if necessary.

10. Good luck, and we look forward to seeing the innovative missions that you come up with!

NOTE: The organizers reserve the right to disqualify any team that does not follow the instructions or that submits a mission that is deemed unsafe or inappropriate for the drone to execute.
We will not be responsible for any damage to the drone or property that results from a poorly designed mission, so please plan your missions carefully and responsibly.
Causing any damage to the drone or property may lead to penalization and teams will be held liable for any such damages. Always prioritize safety in your mission design.
'''


#Import any libraries you need here
#For example, you might want to import math for calculations, or numpy for array manipulations.


def mission_plan():
    
    '''
    This function should return a list of waypoints for the drone to follow.
    ONLY EDIT THIS FUNCTION. DO NOT EDIT ANY OTHER PART OF THE CODE.
    '''
    
    waypoints = []
    
    return waypoints