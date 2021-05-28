import json
from veides_agent.services import (
    ActionCompletedService,
    EventService,
    FactsService,
    MethodCompletedService,
    RegisterMethodService,
    TrailsService
)
from veides_agent_ros.srv import (
    ActionCompleted,
    ActionCompletedRequest,
    Facts,
    FactsRequest,
    Event,
    EventRequest,
    MethodCompleted,
    MethodCompletedRequest,
    RegisterMethod,
    RegisterMethodRequest,
    Trails,
    TrailsRequest
)
from veides_agent_ros.msg import Fact, Trail
from tests.conftest import ros, veides_agent_node, RosMessageMock, RosServiceMock


def test_action_completed_service_id_and_message_type(ros, veides_agent_node):
    service = ActionCompletedService(veides_agent_node, ros)

    assert service.id == 'action_completed'
    assert service.message_type == ActionCompleted


def test_should_send_action_completed(ros, veides_agent_node):
    action_name = 'some_action'

    service = ActionCompletedService(veides_agent_node, ros)

    request = ActionCompletedRequest()
    request.name = action_name

    response = service.on_called(request)

    veides_agent_node.client.send_action_completed.assert_called_once_with(action_name)
    assert response.rc == 0


def test_event_service_id_and_message_type(ros, veides_agent_node):
    service = EventService(veides_agent_node, ros)

    assert service.id == 'event'
    assert service.message_type == Event


def test_should_send_event(ros, veides_agent_node):
    event_name = 'some_event'

    service = EventService(veides_agent_node, ros)

    request = EventRequest()
    request.name = event_name

    response = service.on_called(request)

    veides_agent_node.client.send_event.assert_called_once_with(event_name)
    assert response.rc == 0


def test_facts_service_id_and_message_type(ros, veides_agent_node):
    service = FactsService(veides_agent_node, ros)

    assert service.id == 'facts'
    assert service.message_type == Facts


def test_should_send_facts(ros, veides_agent_node):
    facts = [Fact(name='some', value='value')]
    expected_facts = {'some': 'value'}

    service = FactsService(veides_agent_node, ros)

    request = FactsRequest()
    request.facts = facts

    response = service.on_called(request)

    veides_agent_node.client.send_facts.assert_called_once_with(expected_facts)
    assert response.rc == 0


def test_method_completed_service_id_and_message_type(ros, veides_agent_node):
    service = MethodCompletedService(veides_agent_node, ros)

    assert service.id == 'method_completed'
    assert service.message_type == MethodCompleted


def test_should_send_method_completed(ros, veides_agent_node):
    method_name = 'some_method'
    payload = {'foo': 'bar'}
    code = 200

    service = MethodCompletedService(veides_agent_node, ros)

    request = MethodCompletedRequest()
    request.name = method_name
    request.payload = json.dumps(payload)
    request.code = code

    response = service.on_called(request)

    veides_agent_node.client.send_method_response.assert_called_once_with(method_name, payload, code)
    assert response.rc == 0


def test_register_method_service_id_and_message_type(ros, veides_agent_node):
    service = RegisterMethodService(veides_agent_node, ros)

    assert service.id == 'register_method'
    assert service.message_type == RegisterMethod


def test_should_register_method(ros, veides_agent_node):
    method_name = 'some_method'

    service = RegisterMethodService(veides_agent_node, ros)

    request = RegisterMethodRequest()
    request.name = method_name
    request.data_message = 'tests.conftest.RosMessageMock'
    request.response_service = 'tests.conftest.RosServiceMock'

    response = service.on_called(request)

    id = 'method/{}'.format(method_name)

    veides_agent_node.create_publisher.assert_called_once_with(id, RosMessageMock, queue_size=1)

    id = 'method_completed/{}'.format(method_name)

    veides_agent_node.create_service.assert_called_once()
    assert veides_agent_node.create_service.call_args.args[0] == id
    assert veides_agent_node.create_service.call_args.args[1] == RosServiceMock
    assert callable(veides_agent_node.create_service.call_args.args[2])
    assert response.rc == 0


def test_register_method_on_method_response_requested_should_return_success(ros, veides_agent_node):
    method_name = 'some_method'
    payload = {
        'foo': 'bar'
    }

    service = RegisterMethodService(veides_agent_node, ros, lambda request_already_dict: request_already_dict)

    request = {
        'code': 200,
        **payload
    }

    class ServiceResponseMock(object):
        def __init__(self, rc):
            self.rc = rc

    class ServiceMock(object):
        response_class = ServiceResponseMock

    veides_agent_node.get_service.return_value = ServiceMock()

    response = service.on_method_response_requested(method_name, 'method_completed/{}'.format(method_name), request)

    veides_agent_node.client.send_method_response.assert_called_once()
    assert veides_agent_node.client.send_method_response.call_args.args[0] == method_name
    assert veides_agent_node.client.send_method_response.call_args.args[1] == payload
    assert veides_agent_node.client.send_method_response.call_args.args[2] == 200
    assert response.rc == 0


def test_register_method_should_return_error_when_data_message_class_not_found(ros, veides_agent_node):
    method_name = 'some_method'

    service = RegisterMethodService(veides_agent_node, ros)

    request = RegisterMethodRequest()
    request.name = method_name
    request.data_message = 'tests.conftest.NotExistingClass'
    request.response_service = 'tests.conftest.RosServiceMock'

    response = service.on_called(request)

    assert response.rc == 1


def test_register_method_should_return_error_when_response_service_class_not_found(ros, veides_agent_node):
    method_name = 'some_method'

    service = RegisterMethodService(veides_agent_node, ros)

    request = RegisterMethodRequest()
    request.name = method_name
    request.data_message = 'tests.conftest.RosMessageMock'
    request.response_service = 'tests.conftest.NotExistingClass'

    response = service.on_called(request)

    assert response.rc == 1


def test_trails_service_id_and_message_type(ros, veides_agent_node):
    service = TrailsService(veides_agent_node, ros)

    assert service.id == 'trails'
    assert service.message_type == Trails


def test_should_send_trails(ros, veides_agent_node):
    trail_name = 'some_trail'
    trail_value = 'some_value'

    service = TrailsService(veides_agent_node, ros)

    request = TrailsRequest()
    request.trails = [
        Trail(name=trail_name, value=trail_value)
    ]

    response = service.on_called(request)

    veides_agent_node.client.send_trail.assert_called_once_with(trail_name, trail_value)
    assert response.rc == 0
