# Veides Agent SDK for ROS

This repository contains package that allows ROS developers to easily connect agents and interact with Veides platform.

**Jump to**:

* [Requirements](#Requirements)
* [Installation](#Installation)
* [Parameters](#Parameters)
* [Publish topics](#Publish-topics)
* [Services](#Services)

## Requirements

* [ROS](https://www.ros.org/) with [Python 3](https://wiki.python.org/moin/BeginnersGuide/Download)

NOTE: [ROS Noetic](http://wiki.ros.org/noetic/Installation/Ubuntu) supports Python 3 by default

## Installation

```bash
cd catkin_workspace/src
git clone https://github.com/Veides/veides_agent_ros.git
pip3 install veides-agent-sdk
cd ..
catkin_make
catkin_make install
```

## Parameters

Required parameters:

| Parameter               | Description        |
| ----------------------: | ------------------ |
| veides_client_host      | Host to connect to |
| veides_agent_client_id  | Agent's client id    |
| veides_agent_key        | Agent's key          |
| veides_agent_secret_key | Agent's secret key   |

Optional parameters:

| Property          | Description                                                                             |
| ----------------: | --------------------------------------------------------------------------------------- |
| veides_agent_name | Agent name used to compose publish topics. By default `veides_agent_client_id` is used. |

## Publish topics

### Action received

Used to trigger action received from Veides

```
/veides/agent/${veides_agent_name}/action_received (veides_agent_ros/Action.msg)
```

## Services

### Action completed

After received action is completed, this service needs to be called, so the agent will be able to receive new actions.

```
/veides/agent/${veides_agent_name}/action_completed (veides_agent_ros/ActionCompleted.srv)
```

You can simply call the service with completed action name:

```python
from veides_agent_ros.srv import ActionCompleted

srv = rospy.ServiceProxy('/veides/agent/${veides_agent_name}/action_completed', ActionCompleted)

srv('completed_action_name')
```

### Event

Sends an event.

```
/veides/agent/${veides_agent_name}/event (veides_agent_ros/Event.srv)
```

You can simply call the service with event name:

```python
from veides_agent_ros.srv import Event

srv = rospy.ServiceProxy('/veides/agent/${veides_agent_name}/event', Event)

srv('event_name')
```

### Facts

Sends facts to be processed.

```
/veides/agent/${veides_agent_name}/facts (veides_agent_ros/Facts.srv)
```

Service accepts an array of `(veides_agent_ros/Fact.msg)` messages:

```python
from veides_agent_ros.srv import Facts
from veides_agent_ros.msg import Fact

srv = rospy.ServiceProxy('/veides/agent/${veides_agent_name}/facts', Facts)

srv([
  Fact(name='battery_level', value='low')
])
```

### Trails

Sends trails.

```
/veides/agent/${veides_agent_name}/trails (veides_agent_ros/Trails.srv)
```

Service accepts an array of `(veides_agent_ros/Trail.msg)` messages:

```python
from veides_agent_ros.srv import Trails
from veides_agent_ros.msg import Trail

srv = rospy.ServiceProxy('/veides/agent/${veides_agent_name}/trails', Trails)

srv([
  Trail(name='up_time', value='1')
])
```
