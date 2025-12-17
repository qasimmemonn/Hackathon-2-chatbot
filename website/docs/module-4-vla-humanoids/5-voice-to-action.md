--- 
sidebar_position: 5
---

# Chapter 5: Voice-to-Action and Cognitive Planning

In the previous chapter, we built a system that allows a robot to listen and respond in natural language. We now face the "last mile" problem: how do we turn the LLM's textual response into physical robot actions? A robot that can talk about picking up an apple but can't actually do it is not very useful.

This chapter explores how to bridge the gap from language to action, transforming our conversational agent into a cognitive planner that can execute tasks in the real world.

## The Challenge: From Unstructured Text to Structured Commands

The LLM provides us with unstructured natural language. For example, in response to "Can you get me the apple?", the model might say:
-   "Sure, I will pick up the apple."
-   "Of course, I am getting the red apple from the table."
-   "Acknowledged. Grasping the apple."

How do we reliably convert any of these variations into a structured command like `robot.grasp(target='apple')`? A simple approach might be to search for keywords.

```python
# The brittle, keyword-based approach
if "pick up" in llm_response or "grasp" in llm_response:
    if "apple" in llm_response:
        robot.grasp(target='apple')
```

This is obviously fragile. It would fail if the user said, "Can you get me that red fruit?" and the robot replied, "Yes, I can get it for you." We need a more robust method.

## The Solution: LLM Function Calling

Modern LLMs (like those from OpenAI, Google, and others) provide a powerful feature called **Function Calling**. This is the state-of-the-art solution to our problem.

The concept is simple:
1.  We define a "toolbox"—a list of functions the robot can execute—and describe them to the LLM.
2.  We provide this toolbox as part of the API call.
3.  The LLM can then choose to either respond with a text message OR a structured JSON object requesting to call one of our functions with specific parameters.

This transforms the LLM from a simple text generator into a reasoning engine that can use tools.

### Defining the Robot's "Toolbox"

The `robot_brain_node` is responsible for defining the set of capabilities it wants to expose to the LLM. This is done by creating a schema, often a JSON object, that describes each function.

**Example Toolbox Definition:**
```python
tools = [
    {
        "type": "function",
        "function": {
            "name": "navigate_to_location",
            "description": "Moves the robot to a known, named location in the environment.",
            "parameters": {
                "type": "object",
                "properties": {
                    "location_name": {
                        "type": "string",
                        "description": "The destination, e.g., 'kitchen', 'living room', or 'table'."
                    }
                },
                "required": ["location_name"]
            }
        }
    },
    {
        "type": "function",
        "function": {
            "name": "grasp_object",
            "description": "Picks up a specified object that is within reach.",
            "parameters": {
                "type": "object",
                "properties": {
                    "object_name": {
                        "type": "string",
                        "description": "The name of the object to pick up, e.g., 'red apple' or 'blue cup'."
                    }
                },
                "required": ["object_name"]
            }
        }
    }
]
```
This "toolbox" is now passed to the LLM API along with the main prompt in every turn of the conversation.

## The New "Voice-to-Action" Control Loop

The control loop in our `robot_brain_node` now becomes more sophisticated.

1.  A user says, "Please bring me the apple from the table."
2.  The STT node transcribes this and sends it to the brain node.
3.  The brain node constructs the prompt (Preamble, World State, History, User Text) and calls the `/llm_service`, this time including the `tools` definition.
4.  The LLM receives the prompt. It understands the user's intent requires moving and grasping. Instead of just replying with text, it returns a structured response:
    ```json
    {
      "tool_calls": [{
        "id": "call_1234",
        "type": "function",
        "function": {
          "name": "navigate_to_location",
          "arguments": "{\"location_name\": \"table\"}"
        }
      }]
    }
    ```
5.  The `robot_brain_node` receives this structured response. It sees a request to call a tool.
6.  It parses the `name` ("navigate_to_location") and `arguments` ("table").
7.  It now calls its own internal Python function, `self.execute_navigation("table")`, which in turn might call a ROS 2 action server for the Nav2 stack.
8.  The robot begins moving to the table. The brain node can report back to the LLM, "Executing navigation to table..." to get a human-friendly response like, "On my way to the table."
9.  Once the navigation is complete, the story isn't over. The brain node must now remember the original request and call the LLM *again* with an updated context: "I have arrived at the table. The user wants me to bring them the apple."
10. The LLM, remembering the full context, will now issue a second tool call:
    ```json
    {
      "tool_calls": [{
        "id": "call_5678",
        "type": "function",
        "function": {
          "name": "grasp_object",
          "arguments": "{\"object_name\": \"apple\"}"
        }
      }]
    }
    ```
11. The brain node receives this, calls its internal `self.execute_grasp("apple")` function, and the robot picks up the apple.

## The LLM as a Cognitive Planner

This example demonstrates something profound. The LLM is no longer just a chatbot. It is acting as a **cognitive planner**. It takes a high-level, multi-step goal ("Bring me the apple from the table") and autonomously decomposes it into a sequence of executable steps from its available "toolbox":
1.  `navigate_to_location('table')`
2.  `grasp_object('apple')`
3.  (and presumably, a third step to navigate back to the user)

This ability to plan, execute, and reason is the core of a Vision-Language-Action model. By providing the LLM with a clear description of the robot's perception of the world and a well-defined set of actions it can take, we empower it to formulate and execute complex plans, bridging the gap from a simple user utterance to a sequence of meaningful, physical actions. This sets the stage for our capstone project, where we will integrate all these components into an autonomous humanoid robot.
