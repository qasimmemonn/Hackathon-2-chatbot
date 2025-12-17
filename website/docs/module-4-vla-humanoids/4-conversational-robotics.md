---
sidebar_position: 4
---

# Chapter 4: Integrating GPT Models for Conversational Robotics

Humanoid robots should be able to interact with people naturally. This means moving beyond rigid, pre-programmed voice commands ("Robot, get object A") to fluid, context-aware dialogue. By integrating powerful Large Language Models (LLMs) like GPT-4, we can give our robot a "brain" that allows it to understand, reason, and converse in human language.

This chapter provides a software architecture for building a conversational robotics system using ROS 2. We will break the problem down into a set of communicating nodes that handle speech-to-text, language understanding, and text-to-speech.

## The Conversational System Architecture

A robust conversational system is not a single program, but a pipeline of specialized nodes. This modular approach allows us to swap out components (e.g., use a different speech-to-text engine) without redesigning the entire system.

![Conversational Robotics Architecture](https://i.imgur.com/gY9RzFf.png)

Our architecture consists of four main components:
1.  **Speech-to-Text (STT) Node:** Captures audio from a microphone and transcribes it into text.
2.  **LLM Service Node:** A central service that handles all communication with the external LLM API (e.g., OpenAI).
3.  **Robot Brain Node:** The core logic node that orchestrates the conversation, maintains state, and decides when to speak or act.
4.  **Text-to-Speech (TTS) Node:** Converts text from the robot into audible speech.

Let's explore each component.

### 1. The Speech-to-Text (STT) Node

The STT node's only job is to listen for a user's voice and convert it to text.
-   **Input:** Raw audio stream from a microphone.
-   **Processing:** Uses a transcription model. A great open-source option is **OpenAI's Whisper**, which offers excellent accuracy. The node would continuously listen and, upon detecting a pause in speech, process the preceding audio clip.
-   **Output:** Publishes the transcribed text to a ROS 2 topic, e.g., `/user_transcription` (`std_msgs/String`).

### 2. The LLM Service Node

To avoid scattering API keys and HTTP request logic throughout our system, we centralize all interaction with the LLM into a single, reusable service.

First, we define a custom service file:
**`my_robot_interfaces/srv/LLM.srv`**
```
# The user's prompt and conversation context
string prompt
---
# The LLM's generated response
string response
```
(Remember to re-build your workspace after creating a new interface file.)

The `llm_service_node` then implements this service:
-   It creates a service server listening on `/llm_service`.
-   When it receives a request, its callback function takes the `prompt` string.
-   Inside the callback, it uses a library like `openai` to make a secure API call to a model like `gpt-4`.
-   It waits for the API response and returns the LLM's generated text in the service response field.

### 3. Prompt Engineering: Giving the LLM Context

This is the most important part of the process. We cannot simply send the user's raw text to the LLM. The model needs context to provide a useful, safe, and relevant response. The `robot_brain_node` is responsible for constructing a detailed **prompt** every time it calls the LLM service.

A well-structured prompt for a robot might look like this:

```
# The System Preamble: Sets the persona and capabilities
You are a helpful and safe robot assistant named Unitree H1, currently in a kitchen.
Your capabilities are: navigating to locations, picking up objects, and answering questions.
The objects you can see are: a red apple on the table, a blue cup.
The locations you know are: the kitchen, the living room.
Be concise and clear in your responses.

# The Conversation History: Provides short-term memory
Human: Can you see anything on the table?
Robot: Yes, I see a red apple and a blue cup.
Human: Please pick up the apple for me.

# The final prompt for the LLM would be the entire block of text above.
```

By providing this context—the robot's identity, capabilities, world state, and conversation history—we "ground" the LLM. It now knows what it can do, what it can see, and what was just discussed, leading to far more intelligent responses than if it only received the words "Please pick up the apple for me."

### 4. The Text-to-Speech (TTS) Node

The TTS node is the inverse of the STT node. It provides the robot's voice.
-   **Input:** Subscribes to a ROS 2 topic, e.g., `/robot_speech` (`std_msgs/String`).
-   **Processing:** Uses a text-to-speech engine (e.g., `pyttsx3`, cloud-based services like Amazon Polly or Google TTS) to convert the incoming text string into an audio waveform.
-   **Output:** Plays the audio through the robot's speakers.

### The "Robot Brain" Node: Orchestrating the Dialogue

This central node ties everything together.

**The Control Loop:**
1.  The `robot_brain_node` subscribes to `/user_transcription`.
2.  When it receives a new message from the user, it appends it to its stored conversation history.
3.  It then constructs the full prompt (System Preamble + World State + Conversation History).
4.  It creates a client for the `/llm_service` and calls it with the complete prompt.
5.  It awaits the response from the LLM service.
6.  Upon receiving the LLM's response (e.g., "Of course, I will pick up the red apple."), it appends this to the conversation history.
7.  It **parses the response**. This is a critical step we will explore in the next chapter. For now, let's assume it's just text.
8.  It publishes the response text to the `/robot_speech` topic for the TTS node to speak aloud.

By creating this clean, modular architecture, we have given our robot the ability to have surprisingly natural conversations. The next and final challenge is to turn the LLM's words into actions.
