# Robot RL Agent: Avoiding Raindrops 🌧️🤖

Welcome to the Robot RL Agent project! This repository showcases a reinforcement learning agent trained to navigate and avoid falling raindrops. Below are key insights and considerations from the development process.

>[!WARNING]
> This repository is still under active development.

![image](https://github.com/hi-jin/avoid-the-rain-robot/assets/51053567/eb7ddda7-a8df-4ec2-992c-423a9afdf3d9)

## Environment Details 🌍

### Observation Space 👁️
The agent receives two types of observations:
- **Depth Image**: A visual representation of the environment
- **Last Action**: The previous action taken by the agent

### Action Space 🎮
The agent controls its movement through velocity adjustments:
- **Velocity Range**: -50 to 50
- This allows the agent to move forward, backward, and turn with varying speeds

## Key Insights 🔑

### 1. Multi Vec Environments 🌐

Multi vec environments proved crucial for stable performance:

- ✅ **Stability**: Reduced variance in training results
- ✅ **Consistency**: More reliable agent performance
- ✅ **Efficiency**: Parallel processing for faster training
- ✅ **Generalization**: Improved agent behavior across scenarios

### 2. Reward Shaping 🎯

Reward shaping significantly influenced agent behavior, presenting both benefits and challenges:

#### Benefits:
- ✅ **Alignment**: Proper shaping aligns agent actions with intended goals
- ✅ **Goal-oriented behavior**: Encourages desired outcomes

#### Challenges:
- ⚠️ **Exploitation**: Agents may find unintended ways to maximize rewards
- ⚠️ **Undesired behavior**: Can lead to actions that increase rewards without achieving actual goals

### Addressing Exploitation vs Alignment:
- 🔍 **Careful Design**: Implement thorough reward function testing
- 🔄 **Iterative Refinement**: Continuously adjust rewards to promote desired behaviors
- 🎯 **Clear Objectives**: Define precise goals to avoid misalignment
- 🧪 **Extensive Testing**: Evaluate agent performance in various scenarios

## Design Considerations 🎨

While aiming for more dynamic movement than shown in the video below, I encountered a challenge: without proper reward function design, the agent would often resort to rapidly spinning its wheels without actually making efforts to avoid the rain. This highlighted the critical importance of thoughtful reward shaping to encourage desired behaviors while discouraging unproductive actions.

## Conclusion 🏁

Developing an effective raindrop-avoiding RL agent requires:
1. Optimized environment setup (multi vec)
2. Thoughtful reward shaping
3. Balancing alignment with exploitation prevention

## Contact 📬

For questions or suggestions, please don't hesitate to reach out.

---

*This README incorporates insights from the Robot RL Agent project development.*

---

https://github.com/hi-jin/avoid-the-rain-robot/assets/51053567/a4b70bfc-552e-41da-9435-8b12f3bb2318

![Agent Performance](https://github.com/hi-jin/avoid-the-rain-robot/assets/51053567/0798dd1b-ec4a-40df-bb2d-b8ce7d2c510a)
