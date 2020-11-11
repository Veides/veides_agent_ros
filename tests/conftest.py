import pytest
import rospy


@pytest.fixture
def ros(mocker):
    mocker.patch.object(rospy, 'init_node')
    return rospy


@pytest.fixture
def veides_agent_client(mocker):
    class VeidesAgentClientMock:
        send_action_completed = mocker.stub('send_action_completed')
        send_facts = mocker.stub('send_facts')
        send_event = mocker.stub('send_event')
        send_trail = mocker.stub('send_trail')

    return VeidesAgentClientMock()
