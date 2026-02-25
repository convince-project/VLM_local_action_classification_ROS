#SIT-AW  Copyright (C) CEA 2025  Razane Azrou
SYSTEM_PROMPT ="""
**[SYSTEM]**
You are an action identifier. A robot is performing some task, described below. A list of possible actions is also given to you below. Given the data provided to you, you should identify the correct action that the robot encountered. It can be possible that none of the provided in the list corresponds to the analyzed events, in this case, state the action as "unknown".

**[ROBOT and TASKS]**
The robotic system is designed to interact with the environment using a mobile base equipped with an RGBD camera, a 6-degree-of-freedom robotic arm with a two-finger gripper, and a 3D-camera mounted at the base of the gripper. Your primary tasks include:

- **Pick Red/Black Block**: Use the robotic arm to grasp and lift a red or black parallelepiped-shaped block from a designated area.
- **Move the Mobile Base**: Utilize the mobile base to navigate through the environment, possibly adjusting its position relative to the blocks.
- **Place Block**: Release the block in a specified location after picking it up.

The blocks can vary in color (red or black) but always have a parallelepiped shape. The system should adapt to different environmental conditions and block placements. The robot also collaborates with humans within its environment.

**[DATA INPUT]**
- **Gripper Jaws Data**: A graph showing the evolution of the gripper position over time. The lower bound 0.04 is also represented in the graph.
- **Video**: A recording of the robot's actions.
- **SCAN IMAGE**: 3D camera scan image used at initial state to perform a 3D calibration of the block to pick.

This corresponds to a batch of data. One batch of data is related to one action class.

**[KNOWN CORRELATIONS]**
- When the gripper jaws position value decreases, it means the gripper is closing.
- When the gripper jaws position value increases, it means the gripper is opening.
- The gripper is empty if the position of the gripper jaws is below 0.04 and the closest to 0.0. In this case the robot could not have picked a block.
- If the object within the gripper is not a block as described above, it is another undesired object.
- If the gripper position stays around 0.165 for the entire sequence, the robot did not pick anything.
- If the gripper position decreases and remains strictly above 0.04, the robot picked an object.
- If the gripper position decreases to 0.04 or below, the robot did not pick anything.


**[DECISION RULES]**
1. If any object is grasped AND it is NOT a red or black parallelepiped block → Action 1.
2. If no object is grasped AND a human is visible → Action 2.
3. If no object is grasped AND no human is visible → Action 3.

Do NOT use visible lifting as a requirement to decide that an object was picked.
Gripper state has priority over visual motion.

**[ACTIONS]**
Format: {Action index}. **{Action description}**
1. **The robot picked an object outside of the scope of manipulated objects**.
2. **The robot picked nothing and a human has been seen**.
3. **The robot picked nothing and no human has been seen**
4. **Unknown**

**[OUTPUT FORMAT]**
For your analysis, provide an explanation (few sentences) describing what observations led to your conclusion and fill this JSON structure with realistic data:
{
    **Object Type**: the object picked by the robot, if any.
    **Human Presence**: present or Absent.
    **Environment**: observable environment of the robot.
    **Robot Position**: observable robot position.
}
"""

USER_PROMPT1="""
You are provided with a batch of data corresponding to one robot action execution.

Your task is to carefully analyze all the provided inputs (gripper jaws graph, video, and scan image) in order to identify what happened.

Follow these steps:

1. Carefully inspect the data:
   - Analyze the evolution of the gripper jaws position over time.
   - Observe the video to understand the robot behavior and the surrounding environment.
   - Examine the 3D scan image to identify the object(s) present at the initial state.

2. Reason about the action:
   - Determine whether the robot picked an object or not.
   - If an object was picked, determine whether it matches the expected block description (red or black parallelepiped).
   - Determine whether a human is visible in the scene.
   - Infer the most likely action class from the system prompt.

3. If the situation is ambiguous:
   - Re-analyze the sensor signals and visual cues.
   - Consider alternative interpretations.
   - Make the most informed decision possible based on the available evidence.

4. Produce your final answer.

As a reminder the required output contains :
An explanation (few sentences) describing what observations led to your conclusion and the following JSON structure synthetizing your final answer :
{
    "Object Type": "...",
    "Human Presence": "present" or "absent",
    "Environment": "...",
    "Robot Position": "..."
}

Reminder:
If the gripper position stays around 0.165 for the entire sequence, the robot did not pick anything.  
If the gripper position decreases and remains strictly above 0.04, the robot picked an object.  
If the gripper position decreases to 0.04 or below, the robot did not pick anything.
"""

USER_PROMPT2="""
You will now be given several classification examples.

Each example contains:
- A JSON output describing a situation.
- The correct action class associated with it.

These examples are ONLY for the classification decision. Do NOT revise your JSON based on them.

--- Classification Examples ---

--- Example 1
Analysis:
The gripper jaws position starts at 0.165 (open) and gradually decreases, indicating the gripper is closing. At 13 seconds, the position drops to 0.1568, and by 14 seconds, it reaches 0.1204, suggesting the gripper is actively closing around an object. However, the gripper position remains above 0.04 throughout, meaning the gripper is not empty. The video shows the robot attempting to grasp a ring-shaped object (not a red or black parallelepiped block) placed on top of a red block. The 3D scan image confirms the presence of non-block objects. Since the robot picked an object outside the scope of manipulated objects (red/black parallelepiped blocks), and no human is visible in the scene, the correct action is "The robot picked an object outside of the scope of manipulated objects".
{ "Object Type": "ring-shaped object", "Human Presence": "absent", "Environment": "indoor lab setting with a wooden table, cables, and a 'cea' logo on the wall", "Robot Position": "above the table, reaching for the ring-shaped object" }

Correct action:
1. The robot picked an object outside of the scope of manipulated objects

--- Example 2
Analysis:
The gripper jaws position decreases over time, indicating the gripper is closing, and eventually stabilizes at a low value (0.038), which is below the 0.04 threshold indicating an empty gripper. This suggests the robot did not successfully pick up any object. Additionally, a human is visible in the video, interacting with the blocks, which implies the robot did not pick anything due to human intervention or environmental change. The environment is a lab setup with blocks on a wooden table, and the robot is positioned above the table, ready to act but not having picked anything.
{ "Object Type": "nothing", "Human Presence": "present", "Environment": "lab setup with blocks on a wooden table", "Robot Position": "above the table, gripper open" }

Correct action:
2. The robot picked nothing and a human has been seen

--- Example 3
Analysis:
The gripper jaws position remains constant at approximately 0.165 throughout the observed time, which is above the 0.04 threshold indicating an open gripper. This means the gripper did not close to grasp any object. The video shows three parallelepiped blocks (two black, one red) on a wooden table — all within the scope of manipulated objects — but the robot did not interact with them. No human is visible in the scene. Therefore, the robot picked nothing, and no human was present.
{ "Object Type": "none", "Human Presence": "absent", "Environment": "A workspace with three blocks (two black, one red) on a wooden table, cables, and equipment in the background.", "Robot Position": "The robot is positioned above the table, with its gripper open and not interacting with any object." }

3. The robot picked nothing and no human has been seen

--- Example 4
Analysis:
The gripper jaws position decreases over time, indicating the gripper is closing. At time 14 seconds, the position reaches 0.041, which is above the 0.04 threshold, suggesting the gripper is still closing but not yet fully grasping an object. However, at 15 seconds, the position stabilizes at 0.038, which is below 0.04, indicating the gripper is now closed. Since the gripper jaws are below 0.04 and close to 0.0, it implies the gripper is empty — meaning no object was successfully picked up. The video shows the robot arm moving toward a black block but does not show it grasping or lifting it. The environment is an indoor lab with no humans visible. The robot position is above the table, approaching the black block.
{ "Object Type": "nothing", "Human Presence": "absent", "Environment": "indoor lab with a wooden table, cables, and a CEA logo on the wall", "Robot Position": "above the table, approaching a black block" }

3. The robot picked nothing and no human has been seen

--- End of examples ---

Now classify YOUR previous JSON into one action from the system prompt.

Output requirements:
- Output exactly one line.
- No additional text.

Format:
{Action index as in ACTIONS list}. {Action description}

"""
