from pydoc import locate
import json
from rospy_message_converter import message_converter
from functools import partial
from veides_agent_ros.srv import (
    ActionCompleted,
    ActionCompletedResponse,
    Event,
    EventResponse,
    Facts,
    FactsResponse,
    MethodCompleted,
    MethodCompletedResponse,
    RegisterMethod,
    RegisterMethodResponse,
    Trails,
    TrailsResponse,
)


def service_factory(cls, node, ros):
    return cls(node, ros)


class VeidesService(object):
    id = None
    message_type = None
    ros = None
    client = None
    node = None

    def __init__(self, node, ros, message_to_dict_converter=None):
        self.node = node
        self.ros = ros
        self.client = node.client

        if message_to_dict_converter is None:
            self.message_to_dict_converter = message_converter.convert_ros_message_to_dictionary
        else:
            self.message_to_dict_converter = message_to_dict_converter

    def on_called(self, *_):
        raise NotImplementedError('on_called is not implemented yet')

    def _get_rc(self, success):
        return 0 if success else 1


class FactsService(VeidesService):
    @property
    def id(self):
        return 'facts'

    @property
    def message_type(self):
        return Facts

    def on_called(self, request):
        self.ros.logdebug('Service {} called: {}'.format(self.id, self.message_to_dict_converter(request)))

        facts = self._map_facts(request.facts)

        success = self.client.send_facts(facts)

        return FactsResponse(rc=self._get_rc(success))

    def _map_facts(self, facts):
        result = {}

        for fact in facts:
            result[fact.name] = fact.value

        return result


class ActionCompletedService(VeidesService):
    @property
    def id(self):
        return 'action_completed'

    @property
    def message_type(self):
        return ActionCompleted

    def on_called(self, request):
        self.ros.logdebug('Service {} called: {}'.format(self.id, self.message_to_dict_converter(request)))

        success = self.client.send_action_completed(request.name)

        return ActionCompletedResponse(rc=self._get_rc(success))

    def _map_facts(self, facts):
        result = {}

        for fact in facts:
            result[fact.name] = fact.value

        return result


class TrailsService(VeidesService):
    @property
    def id(self):
        return 'trails'

    @property
    def message_type(self):
        return Trails

    def on_called(self, request):
        self.ros.logdebug('Service {} called: {}'.format(self.id, self.message_to_dict_converter(request)))

        results = []

        for trail in request.trails:
            results.append(self.client.send_trail(trail.name, trail.value))

        return TrailsResponse(rc=self._get_rc(all(results)))


class EventService(VeidesService):
    @property
    def id(self):
        return 'event'

    @property
    def message_type(self):
        return Event

    def on_called(self, request):
        self.ros.logdebug('Service {} called: {}'.format(self.id, self.message_to_dict_converter(request)))

        success = self.client.send_event(request.name)

        return EventResponse(rc=self._get_rc(success))


class MethodCompletedService(VeidesService):
    @property
    def id(self):
        return 'method_completed'

    @property
    def message_type(self):
        return MethodCompleted

    def on_called(self, request):
        self.ros.logdebug('Service {} called: {}'.format(self.id, self.message_to_dict_converter(request)))

        success = self.client.send_method_response(request.name, json.loads(request.payload), request.code)

        return MethodCompletedResponse(rc=self._get_rc(success))


class RegisterMethodService(VeidesService):
    @property
    def id(self):
        return 'register_method'

    @property
    def message_type(self):
        return RegisterMethod

    def on_called(self, request):
        self.ros.logdebug('Service {} called: {}'.format(self.id, self.message_to_dict_converter(request)))

        data_message, response_service = self._locate(request)

        if data_message is None or response_service is None:
            return RegisterMethodResponse(rc=1)

        id = 'method/{}'.format(request.name)

        self.node.create_publisher(
            id,
            data_message,
            queue_size=self.node.method_queue_size
        )

        id = 'method_completed/{}'.format(request.name)

        self.node.create_service(
            id,
            response_service,
            partial(self.on_method_response_requested, request.name, id)
        )

        return RegisterMethodResponse(rc=0)

    def on_method_response_requested(self, name, id, request):
        service = self.node.get_service(id)
        method_response = self.message_to_dict_converter(request)

        payload, code = self._map_method_response(method_response.copy())

        success = self.client.send_method_response(name, payload, code)
        return service.response_class(rc=0 if success else 1)

    def _map_method_response(self, method_response):
        code = method_response.get('code', 200)

        if 'code' in method_response:
            method_response.pop('code')

        return method_response, code

    def _locate(self, request):
        data_message = locate(request.data_message)

        if data_message is None:
            self.ros.logerr('Message {} provided in data_message not found'.format(request.data_message))

        response_service = locate(request.response_service)

        if response_service is None:
            self.ros.logerr('Service {} provided in response_service not found'.format(request.response_service))

        return data_message, response_service
