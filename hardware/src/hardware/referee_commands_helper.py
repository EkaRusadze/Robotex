import rospy
import diploaf_robot.config_client as config_client


class RefereeCommandsHelper(config_client.ConfigClient):
    def __init__(self):
        super(RefereeCommandsHelper, self).__init__()
        self.init_config_client()
        self.field_id = self.config['field_id']
        self.robot_id = self.config['robot_id']

    def conf_callback(self):
        self.field_id = self.config['field_id']
        self.robot_id = self.config['robot_id']

    # a{field_id}{robot_id}{command}
    # commands len should be 9
    # ex. aABSTART----
    def receive_command(self, command):
        recv_command = ""
        if len(command) != 12 or command[0] != 'a':
            rospy.logwarn_throttle(1, msg='Referee: Received wrong referee command')
            return recv_command

        recv_field_id = command[1]
        recv_robot_id = command[2]

        if recv_field_id != self.field_id:
            rospy.loginfo_throttle(1,
                                   msg='Referee: expected field {}, received {}'.format(self.field_id, recv_field_id))

        if recv_robot_id == 'X' or recv_robot_id == self.robot_id:
            recv_command = command[3:11]

        rospy.loginfo('Referee: received command {}'.format(recv_command))
        return recv_command

    def get_command_to_send(self, message):
        command = "{}---------".format(message)
        command = command[0:9]
        return "a{}{}{}".format(self.field_id, self.robot_id, command)
