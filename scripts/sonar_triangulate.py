#! /usr/bin/env python

import math
import rospy
from sonar import triangulate
from au_sonar.srv import Triangulate
from au_sonar.srv import TriangulateResponse
from au_core_utils.load_topics import load_topics

data = []
last_result = (float('nan'), float('nan'))


def triangulate_service(req):
    global last_result
    response = TriangulateResponse()
    response.pinger_location.x = float('nan')
    response.pinger_location.y = float('nan')
    response.pinger_location.z = float('nan')

    if math.isnan(req.position.x) and math.isnan(req.position.y):
        response.pinger_location.x = last_result[0]
        response.pinger_location.y = last_result[1]
    else:
        data.append((req.position.x, req.position.y, req.heading))
        if len(data) == 1:
            print('Need another point before ready to triangulate')

        else:
            intersections, result = triangulate(data)
            print('Pinger location: {}'.format(result))
            print('** intersections:')
            for intersection in intersections:
                print('-> {}, {}'.format(intersection[0], intersection[1]))
            response.pinger_location.x = result[0]
            response.pinger_location.y = result[1]
            last_result = result

        return response


if __name__ == '__main__':
    rospy.init_node('sonar_triangulation')

    topics = load_topics()
    service_topic_name = topics['/topic/sensor/sonar/triangulate']

    s = rospy.Service(service_topic_name, Triangulate, triangulate_service)
    print "Ready to find that pinger. Send me some coods"
    rospy.spin()
