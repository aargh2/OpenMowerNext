---
title: ROS MCP
---
# {{ $frontmatter.title }}

## Overview

OpenMowerNext can expose the running ROS graph to OpenCode through [`robotmcp/ros-mcp-server`](https://github.com/robotmcp/ros-mcp-server). The project configuration starts `ros-mcp` as an MCP stdio server, and `ros-mcp` talks to ROS through `rosbridge` on port `9090`.

The default robot setup exposes rosbridge on `0.0.0.0:9090` so the web UI and LAN MCP clients can connect. Use this only on a trusted network.

## Install dependencies

Install the ROS side:

```bash
make deps
```

If `rosbridge_server` is still missing, install it directly:

```bash
make rosbridge-deps
```

Install `uv` for the OpenCode MCP command if `uvx` is not already available:

```bash
curl -LsSf https://astral.sh/uv/install.sh | sh
```

Restart your shell after installing `uv`, or ensure the `uvx` binary directory is on `PATH` before starting OpenCode.

## Run rosbridge

The main mower launch starts rosbridge automatically:

```bash
ros2 launch open_mower_next openmower.launch.py
```

For foreground development:

```bash
make rosbridge
```

This runs `ros2 launch rosbridge_server rosbridge_websocket_launch.xml` after sourcing ROS 2 Jazzy and the workspace install if it exists. It binds to `0.0.0.0:9090` by default.

For a persistent user service:

```bash
make rosbridge-service-enable
```

Useful service commands:

```bash
make rosbridge-service-status
make rosbridge-service-logs
make rosbridge-service-restart
make rosbridge-service-disable
```

The service also binds to `0.0.0.0:9090` by default. Override the bind address when you need a localhost-only development setup:

```bash
make ROSBRIDGE_ADDRESS=127.0.0.1 rosbridge-service-enable
```

If OpenCode runs on another machine and direct LAN access is not appropriate, use an SSH tunnel instead:

```bash
ssh -L 9090:127.0.0.1:9090 <dev-host>
```

## Codex MCP

The project `.codex/config.toml` configures:

```toml
[mcp_servers.ros_mcp]
command = "uvx"
args = ["ros-mcp", "--transport=stdio"]
enabled = true
```

Codex reads project MCP configuration when a trusted project thread starts. Restart Codex or open a new thread after installing `uv` or changing `.codex/config.toml`.

## OpenCode MCP

The project `opencode.json` configures:

```json
{
  "mcp": {
    "ros-mcp": {
      "type": "local",
      "command": ["uvx", "ros-mcp", "--transport=stdio"]
    }
  }
}
```

OpenCode reads this file only when it starts. Quit and restart OpenCode after changing the config or after installing `uv`.

Once OpenCode is restarted and rosbridge is running, ask it to connect to the robot host on port `9090` and inspect topics or services.
