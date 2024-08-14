#!/usr/bin/env python3
import rospy
from flask import Flask, request, jsonify
from std_msgs.msg import Int32MultiArray

rospy.init_node('my', anonymous=True)

pub = rospy.Publisher('number_array', Int32MultiArray, queue_size=10)

# Corrected: 'pub' is the name of the node
# Create Int32MultiArray message
msg = Int32MultiArray()


app = Flask(__name__)

@app.route('/data', methods=['POST'])
def receive_data():
    data = request.json
    # Process the data as needed
    print("Received data:", data)
    numbers = [int(num) for num in data['room_numbers']]
    print(numbers)  # Output: [1, 2]
    msg.data = numbers  # Assign the array of numbers here
    pub.publish(msg)
    return jsonify({"status": "success"}), 200


if __name__ == '__main__':
    app.run(host='0.0.0.0', port=5000)
    rospy.spin()
