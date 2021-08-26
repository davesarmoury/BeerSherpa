#!/usr/bin/env python

from flask import Flask
from flask import request

import rospy
from std_msgs.msg import String

app = Flask(__name__)

@app.route('/')
def get_message():
    global pub

    cleaned = str(request.data)
    print str(cleaned)
    cleaned = cleaned.upper()
    cleaned = cleaned.replace("THE", "")
    cleaned = cleaned.strip()
    print str(cleaned)
    pub.publish(cleaned)
    return "Done"

def main():
    global pub
    pub = rospy.Publisher('ifttt_data', String, queue_size=10)
    rospy.init_node('ifttt', anonymous=True)
    app.run("0.0.0.0")

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
