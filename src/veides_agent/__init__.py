from veides_agent_ros.msg import Action, ActionEntity
from veides.sdk.agent import AgentClient, ConnectionProperties, AgentProperties
from veides_agent.logger import RosNodeConnectedLogger


class VeidesAgentNode:
    def __init__(self, ros, service_factories):
        ros.init_node('veides_agent_node', log_level=ros.INFO, anonymous=True)

        self._services = []
        self._ros = ros

        self._validate_params()

        host = self._ros.get_param('~host')
        key = self._ros.get_param('~key')
        secret_key = self._ros.get_param('~secret_key')
        client_id = self._ros.get_param('~client_id')
        self.agent_name = self._ros.get_param('~name', client_id)
        capath = self._ros.get_param('~capath', "")

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
        self.client.connect()

        if not self.client.connected.wait(timeout=30):
            raise Exception('Not connected')

        self._create_services(service_factories)
        self._register_services()

        self.action_publisher = self._ros.Publisher('/veides/agent/{}/action_received'.format(self.agent_name), Action, queue_size=5)

    def loop(self):
        self._ros.sleep(0.1)

    def shutdown(self):
        self.client.disconnect()

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

    def _create_services(self, services):
        for service in services:
            self._services.append(service(self._ros, self.client, self.agent_name))

    def _register_services(self):
        for service in self._services:
            self._ros.Service(service.id, service.message_type, service.on_called)

    def _on_action(self, name, entities):
        data = Action()
        data.name = name
        data.entities = [
            ActionEntity(name=entity.get('name'), value=str(entity.get('value'))) for entity in entities
        ]

        self.action_publisher.publish(data)
