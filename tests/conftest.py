import pytest
import rospy


@pytest.fixture
def ros(mocker):
    mocker.patch.object(rospy, 'init_node')
    return rospy


@pytest.fixture
def veides_agent_client(mocker):
    class VeidesAgentClientMock(object):
        send_action_completed = mocker.stub('send_action_completed')
        send_event = mocker.stub('send_event')
        send_facts = mocker.stub('send_facts')
        send_method_response = mocker.stub('send_method_response')
        send_trail = mocker.stub('send_trail')

    return VeidesAgentClientMock()


@pytest.fixture
def veides_agent_node(mocker, veides_agent_client):
    class VeidesAgentNodeMock(object):
        client = veides_agent_client
        create_publisher = mocker.stub('create_publisher')
        create_service = mocker.stub('create_service')
        get_service = mocker.stub('get_service')
        method_queue_size = 1

    return VeidesAgentNodeMock()


class RosMessageMock(object):
    pass


class RosServiceMock(object):
    pass
