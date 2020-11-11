from veides_agent_ros.srv import (
    Facts,
    FactsResponse,
    ActionCompleted,
    ActionCompletedResponse,
    Trails,
    TrailsResponse,
    Event,
    EventResponse
)


def service_factory(cls, ros, client, agent_name):
    return cls(ros, client, agent_name)


class VeidesService(object):
    id = None
    message_type = None
    ros = None
    client = None
    agent_name = ''

    def __init__(self, ros, client, agent_name):
        self.ros = ros
        self.client = client
        self.agent_name = agent_name

    def on_called(self, *_):
        raise NotImplementedError('on_called is not implemented yet')


class FactsService(VeidesService):
    @property
    def id(self):
        return '/veides/agent/{}/facts'.format(self.agent_name)

    @property
    def message_type(self):
        return Facts

    def on_called(self, request):
        self.ros.logdebug('Service called: {}'.format(self.id))

        facts = self._map_facts(request.facts)

        self.client.send_facts(facts)

        return FactsResponse()

    def _map_facts(self, facts):
        result = {}

        for fact in facts:
            result[fact.name] = fact.value

        return result


class ActionCompletedService(VeidesService):
    @property
    def id(self):
        return '/veides/agent/{}/action_completed'.format(self.agent_name)

    @property
    def message_type(self):
        return ActionCompleted

    def on_called(self, request):
        self.ros.logdebug('Service called: {}'.format(self.id))

        self.client.send_action_completed(request.name)

        return ActionCompletedResponse()

    def _map_facts(self, facts):
        result = {}

        for fact in facts:
            result[fact.name] = fact.value

        return result


class TrailsService(VeidesService):
    @property
    def id(self):
        return '/veides/agent/{}/trails'.format(self.agent_name)

    @property
    def message_type(self):
        return Trails

    def on_called(self, request):
        self.ros.logdebug('Service called: {}'.format(self.id))

        for trail in request.trails:
            self.client.send_trail(trail.name, trail.value)

        return TrailsResponse()


class EventService(VeidesService):
    @property
    def id(self):
        return '/veides/agent/{}/event'.format(self.agent_name)

    @property
    def message_type(self):
        return Event

    def on_called(self, request):
        self.ros.logdebug('Service called: {}'.format(self.id))

        self.client.send_event(request.name)

        return EventResponse()
