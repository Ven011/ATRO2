#! /usr/bin/python3

"""Sends serial messages to the arduino for movement
"""

import rospy
from std_msgs.msg import String
from pynput import keyboard

def main():
	rospy.init_node("manual_ATRO")
	cmd_pub = rospy.Publisher("move_cmds", String, queue_size=1)
	cmd = String()

	listener = keyboard.Listener()

	def on_press(key):
		nonlocal cmd, cmd_pub

		# on key press, modify twist message and publish it
		try:
			k = key.char
		except AttributeError:
			k = key.name

		if k == 'left':
			cmd.data = 'l'
		if k == 'right':
			cmd.data = 'r'
		if k == 'up':
			cmd.data = 'f'
		if k == 'down':
			cmd.data = 'b'
		if k == 'home':
			cmd.data = 'I'
		if k == 'end':
			cmd.data = 'D'

		cmd_pub.publish(cmd)

	def on_release(key):
		nonlocal cmd, cmd_pub

		# on key release, zero out that key in the twist message
		try:
			k = key.char
		except AttributeError:
			k = key.name

		if k == 'left' or k == 'right' or k == 'up' or k == 'down':
			cmd.data = 'h'

		cmd_pub.publish(cmd)


	with keyboard.Listener(on_press=on_press, on_release=on_release) as listener:
		listener.join()

	rospy.spin()

if __name__ == "__main__":
	main()

