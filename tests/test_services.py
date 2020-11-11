from veides_agent.services import ActionCompletedService, FactsService, EventService, TrailsService
from veides_agent_ros.srv import (
    ActionCompleted,
    ActionCompletedRequest,
    Facts,
    FactsRequest,
    Event,
    EventRequest,
    Trails,
    TrailsRequest
)
from veides_agent_ros.msg import Fact, Trail
from conftest import ros, veides_agent_client


def test_facts_service_id():
    service = FactsService(None, None, 'agent_name')

    assert service.id == '/veides/agent/agent_name/facts'


def test_facts_service_message_type():
    service = FactsService(None, None, 'agent_name')

    assert service.message_type == Facts


def test_action_completed_service_id():
    service = ActionCompletedService(None, None, 'agent_name')

    assert service.id == '/veides/agent/agent_name/action_completed'


def test_action_completed_service_message_type():
    service = ActionCompletedService(None, None, 'agent_name')

    assert service.message_type == ActionCompleted


def test_should_send_facts(ros, veides_agent_client):
    facts = [Fact(name='some', value='value')]
    expected_facts = {'some': 'value'}

    service = FactsService(ros, veides_agent_client, 'agent_name')

    request = FactsRequest()
    request.facts = facts

    service.on_called(request)

    veides_agent_client.send_facts.assert_called_once_with(expected_facts)


def test_should_send_action_completed(ros, veides_agent_client):
    action_name = 'some_action'

    service = ActionCompletedService(ros, veides_agent_client, 'agent_name')

    request = ActionCompletedRequest()
    request.name = action_name

    service.on_called(request)

    veides_agent_client.send_action_completed.assert_called_once_with(action_name)


def test_should_send_event(ros, veides_agent_client):
    event_name = 'some_event'

    service = EventService(ros, veides_agent_client, 'agent_name')

    request = EventRequest()
    request.name = event_name

    service.on_called(request)

    veides_agent_client.send_event.assert_called_once_with(event_name)


def test_should_send_trails(ros, veides_agent_client):
    trail_name = 'some_trail'
    trail_value = 'some_value'

    service = TrailsService(ros, veides_agent_client, 'agent_name')

    request = TrailsRequest()
    request.trails = [
        Trail(name=trail_name, value=trail_value)
    ]

    service.on_called(request)

    veides_agent_client.send_trail.assert_called_once_with(trail_name, trail_value)
