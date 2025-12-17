# Sub-Agents Documentation

## Overview
Sub-agents are specialized autonomous agents that can handle complex, multi-step tasks independently. Each sub-agent has specific capabilities and tools available to it, allowing for efficient task execution.

## Available Sub-Agents

### 1. General Purpose Agent
- **Type**: general-purpose
- **Capabilities**: Researching complex questions, searching code, executing multi-step tasks
- **Tools**: All available tools
- **Use Cases**: When searching for a keyword or file and uncertain of finding the right match in the first few tries

### 2. Code Explorer Agent
- **Type**: Explore
- **Capabilities**: Fast exploration of codebases, finding files by patterns, searching code for keywords
- **Tools**: All tools
- **Thoroughness Levels**: quick, medium, very thorough
- **Use Cases**: Answering questions about codebase structure, finding API endpoints, understanding how features work

### 3. Planning Agent
- **Type**: Plan
- **Capabilities**: Software architecture design, implementation planning
- **Tools**: All tools
- **Use Cases**: Designing implementation strategies, identifying critical files, considering architectural trade-offs

### 4. Claude Code Guide Agent
- **Type**: claude-code-guide
- **Capabilities**: Answering questions about Claude Code CLI, Claude Agent SDK, Claude API
- **Tools**: Glob, Grep, Read, WebFetch, WebSearch
- **Use Cases**: Questions about Claude Code features, hooks, slash commands, MCP servers, settings, etc.

### 5. Status Line Setup Agent
- **Type**: statusline-setup
- **Capabilities**: Configuring Claude Code status line settings
- **Tools**: Read, Edit
- **Use Cases**: Setting up user's Claude Code status line

## Sub-Agent Invocation
Sub-agents are launched through the Task tool with the appropriate subagent_type parameter. They work autonomously to complete complex tasks and return results when finished.

## Best Practices
- Use the most appropriate sub-agent for the specific task
- Launch multiple agents concurrently when possible for optimal performance
- Use run_in_background parameter for long-running agents
- Resume agents using agent ID when continuing previous work
- Provide clear, detailed prompts for autonomous operation