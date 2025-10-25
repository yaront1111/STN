# Claude Code Configuration for CHIMERA Project

This directory contains custom configuration for Claude Code behavior in this project.

## Project Context

**CHIMERA** - Multi-sensor fusion navigation stack for UAVs
- IMU preintegration (GTSAM)
- Magnetometer yaw correction
- Optical flow velocity estimation
- LiDAR/Barometer altitude
- Airspeed + wind estimation

## Key Files

- `commands/` - Custom slash commands for this project
- `project-context.md` - Project-specific instructions for Claude

## Custom Commands

Use these with `/command-name`:

- `/test-build` - Build and run unit tests
- `/test-drift` - Run multi-sensor node and check drift statistics
- `/review-sensor` - Review sensor fusion code for a specific sensor type
- `/check-units` - Verify sensor units (rad/s, m/s², etc.)

## Adding New Commands

Create a `.md` file in `commands/` directory:

```bash
echo "Your instruction here" > .claude/commands/my-command.md
```
Use Gemini as a coding partner via the multi-agent framework.

Steps:
1. Activate the multi-agent environment located at D:\projexts\multi-agent
2. Send the current task context to Gemini for collaborative problem-solving
3. Areas where Gemini excels (use for these):
   - Alternative algorithm suggestions
   - Mathematical derivations (Jacobians, covariances)
   - Code optimization ideas
   - Literature/research references
4. Share relevant code snippets and current challenges
5. Synthesize Gemini's suggestions with my implementation
6. Implement the best combined approach

When to use:
- Complex mathematical problems (factor Jacobians, covariance propagation)
- Need alternative perspectives on architecture
- Research paper implementation guidance
- Performance optimization strategies

Context to share with Gemini:
- Current file being worked on
- Specific problem (units, divergence, etc.)
- Constraints (GTSAM, real-time performance)
- Failed approaches tried
Then use it with `/my-command`
