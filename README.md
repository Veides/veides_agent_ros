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

| Parameter                | Description                                    |
| -----------------------: | ---------------------------------------------- |
| veides_client_host       | Host to connect to                             |
| veides_agent_client_id   | Agent's client id                              |
| veides_agent_key         | Agent's key                                    |
| veides_agent_secret_key  | Agent's secret key                             |

Optional parameters:

| Property                 | Description                                                                             |
| -----------------------: | --------------------------------------------------------------------------------------- |
| veides_agent_name        | Agent name used to compose publish topics. By default `veides_agent_client_id` is used. |
| veides_client_capath     | Path to certificates directory.                                                         |
| veides_action_queue_size | Queue size used by `Action received` publisher. Default value is `veides_queue_size`.   |
| veides_method_queue_size | Queue size used by `Method invoked` publisher. Default value is `veides_queue_size`.    |
| veides_queue_size        | Fallback queue size for publishers. Default value is `5`.                               |

## Publish topics

### Action received

Used to trigger action received from Veides

```
/veides/agent/${veides_agent_name}/action_received (veides_agent_ros/Action.msg)
```

### Method invoked

Used to handle the method invoked by user

```
/veides/agent/${veides_agent_name}/method_invoked (veides_agent_ros/Method.msg)
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

response = srv('completed_action_name')

assert response.rc == 0
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

response = srv('event_name')

assert response.rc == 0
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

response = srv([
  Fact(name='battery_level', value='low')
])

assert response.rc == 0
```

### Method completed

Sends response to the method invoked.

```
/veides/agent/${veides_agent_name}/method_completed (veides_agent_ros/MethodCompleted.srv)
```

Service accepts method response code of type `uint16`, the name of method of type `string` and payload of type `string` as well:

```python
from veides_agent_ros.srv import MethodCompleted

srv = rospy.ServiceProxy('/veides/agent/${veides_agent_name}/method_completed', MethodCompleted)

response = srv(
  name='shutdown',
  payload='{}',
  code=200
)

assert response.rc == 0
```

### Register method [experimental]

In case you want to use your custom ROS messages to deal with invoked methods in a typed way, use this service to register your custom method messages.

```
/veides/agent/${veides_agent_name}/register_method (veides_agent_ros/RegisterMethod.srv)
```

Service accepts method name of type `string`, python-like path to message class published on method invoke and python-like path to service class used to send response for particular method:

```python
from veides_agent_ros.srv import RegisterMethod

srv = rospy.ServiceProxy(f'/veides/agent/{veides_agent_name}/register_method', RegisterMethod)

# ShutdownMethod.msg
# string time
# 
# ShutdownMethodCompleted.srv
# string time
# uint16 code
# ---
# uint8 rc

response = srv(
  name='shutdown',
  data_message='my_package.msg.ShutdownMethod',
  response_service='my_package.srv.ShutdownMethodCompleted'
)

assert response.rc == 0
```

After service is called and the classes were found under provided paths, new publisher and service is created to handle this method. Both may be used as follows:

```python
from my_package.msg import ShutdownMethod
from my_package.srv import ShutdownMethodCompleted

rospy.wait_for_service(f'/veides/agent/{veides_agent_name}/method_completed/shutdown')
srv = rospy.ServiceProxy(f'/veides/agent/{veides_agent_name}/method_completed/shutdown', ShutdownMethodCompleted)

def on_shutdown_method(shutdown):
  response = srv(time=shutdown.time, code=200)
  assert response.rc == 0

rospy.Subscriber(f'/veides/agent/{veides_agent_name}/method/shutdown', ShutdownMethod, on_shutdown_method)
```

Service must define `rc` field of type `uint8` in it's response and the `code` field definition is optional (`200` when not provided).

NOTE: If method is registered to be used with custom messages, it won't be published on `Method invoked` topic.

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
