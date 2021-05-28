import json
from veides_agent_ros.msg import Action, ActionEntity, Method
from veides.sdk.agent import AgentClient, ConnectionProperties, AgentProperties
from veides_agent.logger import RosNodeConnectedLogger


class VeidesAgentNode:
    def __init__(self, ros, service_factories):
        ros.init_node('veides_agent_node', log_level=ros.INFO, anonymous=True)

        self._services = {}
        self._publishers = {}
        self._ros = ros

        self._validate_params()

        host = self._ros.get_param('~host')
        key = self._ros.get_param('~key')
        secret_key = self._ros.get_param('~secret_key')
        client_id = self._ros.get_param('~client_id')
        self.agent_name = self._ros.get_param('~name', client_id)
        capath = self._ros.get_param('~capath', "")
        self.default_queue_size = int(self._ros.get_param('~queue_size', 5))
        self.action_queue_size = int(self._ros.get_param('~action_queue_size', self.default_queue_size))
        self.method_queue_size = int(self._ros.get_param('~method_queue_size', self.default_queue_size))

        connection_propeties = {
            'host': host
        }

        if isinstance(capath, str) and len(capath) > 0:
            connection_propeties['capath'] = capath

        self.client = AgentClient(
            connection_properties=ConnectionProperties(**connection_propeties),
            agent_properties=AgentProperties(
                client_id=client_id,
                key=key,
                secret_key=secret_key
            ),
            logger=RosNodeConnectedLogger(self._ros, '[veides_agent_sdk]'),
            mqtt_logger=RosNodeConnectedLogger(self._ros, '[veides_agent_sdk/Paho]')
        )

        self.client.on_any_action(self._on_action)
        self.client.on_any_method(self._on_method)

        self.client.connect()

        if not self.client.connected.wait(timeout=30):
            raise Exception('Not connected')

        self._create_services(service_factories)

        self.action_publisher = self.create_publisher('action_received', Action, self.action_queue_size)
        self.method_publisher = self.create_publisher('method_invoked', Method, self.method_queue_size)

    def loop(self):
        self._ros.sleep(0.1)

    def shutdown(self):
        self.client.disconnect()

    def create_publisher(self, id, message_class, queue_size=None):
        if queue_size is None:
            queue_size = self.default_queue_size

        topic = self._build_topic(id)
        publisher = self._ros.Publisher(topic, message_class, queue_size=queue_size)
        self._publishers[topic] = publisher
        return publisher

    def get_publisher(self, id):
        return self._publishers.get(self._build_topic(id), None)

    def create_service(self, id, service_class, callback):
        topic = self._build_topic(id)
        service = self._ros.Service(topic, service_class, callback)
        self._services[topic] = service
        return service

    def get_service(self, id):
        return self._services.get(self._build_topic(id), None)

    def _validate_params(self):
        if not self._ros.has_param('~host'):
            self._ros.logerr("Missing host")
            raise ValueError('Missing host')

        if not self._ros.has_param('~client_id'):
            self._ros.logerr("Missing client ID")
            raise ValueError('Missing client ID')

        if not self._ros.has_param('~key'):
            self._ros.logerr("Missing key")
            raise ValueError('Missing key')

        if not self._ros.has_param('~secret_key'):
            self._ros.logerr("Missing secret key")
            raise ValueError('Missing secret key')

    def _create_services(self, factories):
        for factory in factories:
            service = factory(self, self._ros)
            self.create_service(service.id, service.message_type, service.on_called)

    def _build_topic(self, id):
        prefix = '/veides/agent/{}'.format(self.agent_name)

        return '{}/{}'.format(prefix, id)

    def _on_action(self, name, entities):
        data = Action()
        data.name = name
        data.entities = [
            ActionEntity(name=entity.get('name'), value=str(entity.get('value'))) for entity in entities
        ]

        self.action_publisher.publish(data)

    def _on_method(self, name, payload):
        publisher = self.get_publisher('method/{}'.format(name))

        if publisher is not None:
            publisher.publish(**payload)
        else:
            self.method_publisher.publish(Method(name=name, payload=json.dumps(payload)))
